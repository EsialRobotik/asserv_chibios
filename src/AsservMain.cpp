#include "AsservMain.h"
#include "ch.h"
#include "hal.h"
#include "commandManager/CommandManager.h"
#include "USBStream.h"
#include "util/constants.h"
#include "Odometry.h"
#include "SlopeFilter.h"
#include "Pll.h"
#include <chprintf.h>
#include "Encoders/Encoder.h"



#define ASSERV_THREAD_PERIOD_MS (2)
#define ASSERV_THREAD_PERIOD_S (float(ASSERV_THREAD_PERIOD_MS)/1000.0)
#define ASSERV_POSITION_DIVISOR (5)

AsservMain::AsservMain(float wheelRadius_mm, float encoderWheelsDistance_mm, float encodersTicksByTurn,
		CommandManager &commandManager, MotorController &motorController, Encoders &encoders, Odometry &odometrie,
		Regulator &angleRegulator, Regulator &distanceRegulator,
		SlopeFilter &angleRegulatorSlopeFilter, SlopeFilter &distanceRegulatorSlopeFilter,
		SpeedController &speedControllerRight, SpeedController &speedControllerLeft,
		Pll &rightPll, Pll &leftPll):

m_motorController(motorController),
m_encoders(encoders),
m_odometry(odometrie),
m_speedControllerRight(speedControllerRight),
m_speedControllerLeft(speedControllerLeft),
m_angleRegulator(angleRegulator),
m_distanceRegulator(distanceRegulator),
m_angleRegulatorSlopeFilter(angleRegulatorSlopeFilter),
m_distanceRegulatorSlopeFilter(distanceRegulatorSlopeFilter),
m_commandManager(commandManager),
m_pllRight(rightPll),
m_pllLeft(leftPll),
m_distanceByEncoderTurn_mm(M_2PI*wheelRadius_mm),
m_encodersTicksByTurn(encodersTicksByTurn),
m_encodermmByTicks(m_distanceByEncoderTurn_mm/m_encodersTicksByTurn),
m_encoderWheelsDistance_ticks(encoderWheelsDistance_mm / m_encodermmByTicks)
{
	m_asservCounter = 0;
	m_distRegulatorOutputSpeedConsign = 0;
	m_angleRegulatorOutputSpeedConsign = 0;
	m_distSpeedLimited = 0;
	m_angleSpeedLimited = 0;
	m_enableMotors = true;
	m_enablePolar = true;
}

float AsservMain::convertSpeedTommSec(float speed_ticksPerSec)
{
	float speed_nbTurnPerSec = speed_ticksPerSec / m_encodersTicksByTurn;

	// vitesse en mm/sec
	return speed_nbTurnPerSec*m_distanceByEncoderTurn_mm;
}

float AsservMain::estimateDeltaAngle(int16_t deltaCountRight, int16_t deltaCountLeft )
{
	// en rad
    return float(deltaCountRight-deltaCountLeft)  / m_encoderWheelsDistance_ticks ;
}

float AsservMain::estimateDeltaDistance(int16_t deltaCountRight, int16_t deltaCountLeft )
{
	// en mm
    return float(deltaCountRight+deltaCountLeft) * (1.0/2.0) * m_encodermmByTicks;
}

void AsservMain::mainLoop()
{
	systime_t time = chVTGetSystemTime();
	time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	while (true)
	{
		int16_t encoderDeltaRight;
		int16_t encoderDeltaLeft;
		m_encoders.getValuesAndReset(&encoderDeltaRight, &encoderDeltaLeft);

		// Mise à jour de la position en polaire
		m_odometry.refresh(
				encoderDeltaRight*m_encodermmByTicks,
				encoderDeltaLeft*m_encodermmByTicks);


		// Estimation & mise à jour des feedbacks
		float deltaAngle_radian = estimateDeltaAngle(encoderDeltaRight, encoderDeltaLeft);
		float deltaDistance_mm = estimateDeltaDistance(encoderDeltaRight, encoderDeltaLeft);

		m_angleRegulator.updateFeedback( deltaAngle_radian);
		m_distanceRegulator.updateFeedback( deltaDistance_mm);


		/* Calculer une nouvelle consigne de vitesse a chaque  ASSERV_POSITION_DIVISOR tour de boucle
		 * L'asserv en vitesse étant commandé par l'asserv en position, on laisse qq'e tours de boucle
		 * à l'asserv en vitesse pour atteindre sa consigne.
		 */
		if( m_asservCounter == ASSERV_POSITION_DIVISOR && m_enablePolar)
		{
			m_commandManager.update( m_odometry.getX(), m_odometry.getY(), m_odometry.getTheta() );

			float angleConsign = m_angleRegulator.updateOutput( m_commandManager.getAngleGoal());
			float distanceConsign = m_distanceRegulator.updateOutput( m_commandManager.getDistanceGoal());

			setRegulatorsSpeed(distanceConsign, angleConsign);

			m_asservCounter=0;
		}

		/*
		 * Regulation en vitesse
		 */
		m_pllRight.update(encoderDeltaRight, ASSERV_THREAD_PERIOD_S );
		float estimatedSpeedRight = convertSpeedTommSec(m_pllRight.getSpeed());

		m_pllLeft.update(encoderDeltaLeft, ASSERV_THREAD_PERIOD_S );
		float estimatedSpeedLeft = convertSpeedTommSec(m_pllLeft.getSpeed());

		// On limite l'acceleration sur la sortie du regulateur de distance et d'angle
		m_distSpeedLimited = m_distanceRegulatorSlopeFilter.filter(ASSERV_THREAD_PERIOD_S, m_distRegulatorOutputSpeedConsign);
		m_angleSpeedLimited = m_angleRegulatorSlopeFilter.filter(ASSERV_THREAD_PERIOD_S, m_angleRegulatorOutputSpeedConsign);

		// Mise à jour des consignes en vitesse avec acceleration limitée
		m_speedControllerRight.setSpeedGoal(m_distSpeedLimited+m_angleSpeedLimited);
		m_speedControllerLeft.setSpeedGoal(m_distSpeedLimited-m_angleSpeedLimited);

		float outputSpeedRight = m_speedControllerRight.update(estimatedSpeedRight);
		float outputSpeedLeft = m_speedControllerLeft.update(estimatedSpeedLeft);

		if(m_enableMotors)
		{
			m_motorController.setMotor2Speed(outputSpeedRight);
			m_motorController.setMotor1Speed(outputSpeedLeft);
		}
		else
		{
			m_motorController.setMotor2Speed(0);
			m_motorController.setMotor1Speed(0);
		}

		USBStream::instance()->setSpeedEstimatedRight(estimatedSpeedRight);
		USBStream::instance()->setSpeedEstimatedLeft(estimatedSpeedLeft);
		USBStream::instance()->setSpeedGoalRight(m_speedControllerRight.getSpeedGoal());
		USBStream::instance()->setSpeedGoalLeft(m_speedControllerLeft.getSpeedGoal());
		USBStream::instance()->setSpeedOutputRight(outputSpeedRight);
		USBStream::instance()->setSpeedOutputLeft(outputSpeedLeft);
		USBStream::instance()->setSpeedIntegratedOutputRight(m_speedControllerRight.getIntegratedOutput());
		USBStream::instance()->setSpeedIntegratedOutputLeft(m_speedControllerLeft.getIntegratedOutput());


		USBStream::instance()->setAngleGoal(m_commandManager.getAngleGoal());
		USBStream::instance()->setAngleAccumulator(m_angleRegulator.getAccumulator());
		USBStream::instance()->setAngleOutput(m_angleRegulatorOutputSpeedConsign);
		USBStream::instance()->setAngleOutputLimited(m_angleSpeedLimited);


		USBStream::instance()->setDistGoal(m_commandManager.getDistanceGoal() );
		USBStream::instance()->setDistAccumulator(m_distanceRegulator.getAccumulator());
		USBStream::instance()->setDistOutput(m_distRegulatorOutputSpeedConsign);
		USBStream::instance()->setDistOutputLimited(m_distSpeedLimited);

		USBStream::instance()->setOdoX(m_odometry.getX());
		USBStream::instance()->setOdoY(m_odometry.getY());
		USBStream::instance()->setOdoTheta(m_odometry.getTheta());

		USBStream::instance()->setRawEncoderDeltaLeft((float)encoderDeltaLeft);
		USBStream::instance()->setRawEncoderDeltaRight((float)encoderDeltaRight);

		USBStream::instance()->SendCurrentStream();

		m_asservCounter++;

		chThdSleepUntil(time);
		time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	}
}

void AsservMain::setRegulatorsSpeed(float distSpeed, float angleSpeed)
{
	m_distRegulatorOutputSpeedConsign = distSpeed;
	m_angleRegulatorOutputSpeedConsign = angleSpeed;
}

void AsservMain::enableMotors(bool enable)
{
	m_enableMotors = enable;
	if(enable)
	{
		// Ici, il faut reset les intégrateurs des asserv en vitesse
		m_speedControllerLeft.resetIntegral();
		m_speedControllerRight.resetIntegral();
	}
}

void AsservMain::enablePolar(bool enable)
{
	m_enablePolar = enable;
}

void AsservMain::setDistSlope(float slope)
{
	m_distanceRegulatorSlopeFilter.setSlope(slope);
}

void AsservMain::setAngleSlope(float slope)
{
	m_angleRegulatorSlopeFilter.setSlope(slope);
}


