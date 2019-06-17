#include "AsservMain.h"
#include "ch.h"
#include "hal.h"
#include "commandManager/CommandManager.h"
#include "USBStream.h"
#include "util/constants.h"
#include "Encoders.h"
#include "Odometry.h"



#define ASSERV_THREAD_PERIOD_MS (5)
#define ASSERV_THREAD_PERIOD_S (float(ASSERV_THREAD_PERIOD_MS)/1000.0)
#define ASSERV_POSITION_DIVISOR (5)

AsservMain::AsservMain(float wheelRadius_mm, float encoderWheelsDistance_mm,
		CommandManager &commandManager, MotorController &motorController, Encoders &encoders, Odometry &odometrie,
		Regulator &angleRegulator, Regulator &distanceRegulator,
		SpeedController &speedControllerRight, SpeedController &speedControllerLeft ):
m_motorController(motorController),
m_encoders(encoders),
m_odometry(odometrie),
m_speedControllerRight(speedControllerRight),
m_speedControllerLeft(speedControllerLeft),
m_angleRegulator(angleRegulator),
m_distanceRegulator(distanceRegulator),
m_commandManager(commandManager)
{
	m_distanceByEncoderTurn_mm = M_2PI*wheelRadius_mm;
	m_encodermmByTicks = m_distanceByEncoderTurn_mm/4096.0;
	m_encoderWheelsDistance_ticks = encoderWheelsDistance_mm / m_encodermmByTicks;
	m_asservCounter = 0;
	m_enableMotors = true;
	m_enablePolar = true;
}


float AsservMain::estimateSpeed(int16_t deltaCount)
{
	const float ticksByTurn = 1024*4;
	const float dt = ASSERV_THREAD_PERIOD_S;

	float deltaAngle_nbTurn = ((float)deltaCount/(float)ticksByTurn);
	float speed_nbTurnPerSec = deltaAngle_nbTurn / dt;

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
		int16_t m_encoderDeltaRight;
		int16_t m_encoderDeltaLeft;
		m_encoders.getValuesAndReset(&m_encoderDeltaRight, &m_encoderDeltaLeft);

		// Mise à jour de la position en polaire
		m_odometry.refresh(
				m_encoderDeltaRight*m_encodermmByTicks,
				m_encoderDeltaLeft*m_encodermmByTicks);


		// Estimation & mise à jour des feedbacks
		float deltaAngle_radian = estimateDeltaAngle(m_encoderDeltaRight, m_encoderDeltaLeft);
		float deltaDistance_mm = estimateDeltaDistance(m_encoderDeltaRight, m_encoderDeltaLeft);

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

			USBStream::instance()->setAngleOutput(angleConsign);
			USBStream::instance()->setDistOutput(distanceConsign);

			setMotorsSpeed(
					distanceConsign-angleConsign,
					distanceConsign+angleConsign);
			m_asservCounter=0;
		}

		// Regulation en vitesse
		float estimatedSpeedRight = estimateSpeed(m_encoderDeltaRight);
		float estimatedSpeedLeft = estimateSpeed(m_encoderDeltaLeft);

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
		USBStream::instance()->setLimitedSpeedGoalRight(m_speedControllerRight.getLimitedSpeedGoal());
		USBStream::instance()->setLimitedSpeedGoalLeft(m_speedControllerLeft.getLimitedSpeedGoal());

		USBStream::instance()->setAngleGoal(m_commandManager.getAngleGoal());
		USBStream::instance()->setAngleAccumulator(m_angleRegulator.getAccumulator());

		USBStream::instance()->setDistGoal(m_commandManager.getDistanceGoal() );
		USBStream::instance()->setDistAccumulator(m_distanceRegulator.getAccumulator());

		USBStream::instance()->setOdoX(m_odometry.getX());
		USBStream::instance()->setOdoY(m_odometry.getY());
		USBStream::instance()->setOdoTheta(m_odometry.getTheta());

		USBStream::instance()->setRawEncoderDeltaLeft(1664);
		USBStream::instance()->setRawEncoderDeltaRight(51);

		USBStream::instance()->SendCurrentStream();

		m_asservCounter++;
		chThdSleepUntil(time);
		time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	}
}

void AsservMain::setMotorsSpeed(float motorLeft, float motorRight)
{
	m_speedControllerRight.setSpeedGoal(motorRight);
	m_speedControllerLeft.setSpeedGoal(motorLeft);
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


