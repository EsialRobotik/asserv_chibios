#include "AsservMain.h"
#include "ch.h"
#include "hal.h"
#include "commandManager/CommandManager.h"
#include "Odometry.h"
#include "SpeedController/SpeedController.h"
#include "Pll.h"
#include "Regulator.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiterInterface.h"
#include "Encoders/Encoder.h"
#include "blockingDetector/BlockingDetector.h"
#include "util/asservMath.h"
#include <chprintf.h>
#include <cfloat>
#include "sampleStream/SampleStreamInterface.h"


AsservMain::AsservMain(uint16_t loopFrequency, uint16_t speedPositionLoopDivisor, float wheelRadius_mm,
        float encoderWheelsDistance_mm, uint32_t encodersTicksByTurn, CommandManager &commandManager,
        MotorController &motorController, Encoders &encoders, Odometry &odometrie,
        Regulator &angleRegulator, Regulator &distanceRegulator,
        AccelerationDecelerationLimiterInterface &angleRegulatorAccelerationLimiter, AccelerationDecelerationLimiterInterface &distanceRegulatorAccelerationLimiter,
        SpeedController &speedControllerRight, SpeedController &speedControllerLeft,
        Pll &rightPll, Pll &leftPll,
        BlockingDetector *blockingDetector) :

        m_motorController(motorController), m_encoders(encoders), m_odometry(odometrie),
            m_speedControllerRight(speedControllerRight), m_speedControllerLeft(speedControllerLeft),
            m_angleRegulator(angleRegulator), m_distanceRegulator(distanceRegulator),
            m_angleRegulatorAccelerationLimiter(angleRegulatorAccelerationLimiter), m_distanceRegulatorAccelerationLimiter(distanceRegulatorAccelerationLimiter),
            m_commandManager(commandManager),
            m_pllRight(rightPll), m_pllLeft(leftPll),
            m_blockingDetector(blockingDetector),
            m_distanceByEncoderTurn_mm(M_2PI * wheelRadius_mm), m_encodersTicksByTurn(encodersTicksByTurn), m_encodermmByTicks(m_distanceByEncoderTurn_mm / m_encodersTicksByTurn),
            m_encoderWheelsDistance_mm(encoderWheelsDistance_mm), m_encoderWheelsDistance_ticks(encoderWheelsDistance_mm / m_encodermmByTicks),
            m_loopFrequency(loopFrequency), m_loopPeriod(1.0 / float(loopFrequency)), m_speedPositionLoopDivisor( speedPositionLoopDivisor)
{
    m_asservCounter = 0;
    m_distRegulatorOutputSpeedConsign = 0;
    m_angleRegulatorOutputSpeedConsign = 0;
    m_distSpeedLimited = 0;
    m_angleSpeedLimited = 0;
    m_enableMotors = true;
    m_enablePolar = true;
    m_asservMode = normal_mode;
    m_directSpeedMode_rightWheelSpeed = 0;
    m_directSpeedMode_leftWheelSpeed = 0;
}

void AsservMain::setEncodersWheelsDistance_mm(float wheelsDistance_mm)
{
	m_encoderWheelsDistance_mm = wheelsDistance_mm;
	m_encoderWheelsDistance_ticks = m_encoderWheelsDistance_mm/m_encodermmByTicks;
	m_odometry.setEncoderWheelsDistance(m_encoderWheelsDistance_mm);
}

float AsservMain::convertSpeedTommSec(float speed_ticksPerSec)
{
    float speed_nbTurnPerSec = speed_ticksPerSec / m_encodersTicksByTurn;

    // vitesse en mm/sec
    return speed_nbTurnPerSec * m_distanceByEncoderTurn_mm;
}

float AsservMain::estimateDeltaAngle(int16_t deltaCountRight, int16_t deltaCountLeft)
{
    // en rad
    return float(deltaCountRight - deltaCountLeft) / m_encoderWheelsDistance_ticks;
}

float AsservMain::estimateDeltaDistance(int16_t deltaCountRight, int16_t deltaCountLeft)
{
    // en mm
    return float(deltaCountRight + deltaCountLeft) * (1.0 / 2.0) * m_encodermmByTicks;
}

void AsservMain::mainLoop()
{
    m_motorController.setMotorRightSpeed(0.0);
    m_motorController.setMotorLeftSpeed(0.0);

    const time_usecs_t loopPeriod_us = (m_loopPeriod * 1000000.0);
    systime_t time = chVTGetSystemTime();
    time += TIME_US2I(loopPeriod_us);

    float polarRegulationPeriod = m_loopPeriod*m_speedPositionLoopDivisor;

    while (true) {
        float encoderDeltaRight;
        float encoderDeltaLeft;
        m_encoders.getValues(&encoderDeltaRight, &encoderDeltaLeft);

        // Mise à jour de la position en polaire
        m_odometry.refresh(encoderDeltaRight * m_encodermmByTicks, encoderDeltaLeft * m_encodermmByTicks);

        // Estimation & mise à jour des feedbacks
        float deltaAngle_radian = estimateDeltaAngle(encoderDeltaRight, encoderDeltaLeft);
        float deltaDistance_mm = estimateDeltaDistance(encoderDeltaRight, encoderDeltaLeft);

        m_angleRegulator.updateFeedback(deltaAngle_radian);
        m_distanceRegulator.updateFeedback(deltaDistance_mm);

        // Estimation des vitesses de chaque roue
        m_pllRight.update(encoderDeltaRight, m_loopPeriod);
        float estimatedSpeedRight = convertSpeedTommSec(m_pllRight.getSpeed());

        m_pllLeft.update(encoderDeltaLeft, m_loopPeriod);
        float estimatedSpeedLeft = convertSpeedTommSec(m_pllLeft.getSpeed());


        /* Calculer une nouvelle consigne de vitesse a chaque  ASSERV_POSITION_DIVISOR tour de boucle
         * L'asserv en vitesse étant commandé par l'asserv en position, on laisse qq'e tours de boucle
         * à l'asserv en vitesse pour atteindre sa consigne.
         */
        if (m_asservCounter == m_speedPositionLoopDivisor && m_enablePolar) {
            m_commandManager.update(m_odometry.getX(), m_odometry.getY(), m_odometry.getTheta());

            if (m_asservMode == normal_mode)
            {
                m_angleRegulatorOutputSpeedConsign = m_angleRegulator.updateOutput( m_commandManager.getAngleGoal() );
                m_distRegulatorOutputSpeedConsign  = m_distanceRegulator.updateOutput( m_commandManager.getDistanceGoal() );

                // On limite l'acceleration sur la sortie du regulateur de distance et d'angle
                m_distSpeedLimited  = m_distanceRegulatorAccelerationLimiter.limitAcceleration(polarRegulationPeriod, m_distRegulatorOutputSpeedConsign, (estimatedSpeedRight+estimatedSpeedLeft)*0.5,  m_commandManager.getDistanceGoal(), m_distanceRegulator.getError());
                m_angleSpeedLimited = m_angleRegulatorAccelerationLimiter.limitAcceleration(polarRegulationPeriod, m_angleRegulatorOutputSpeedConsign, (estimatedSpeedRight-estimatedSpeedLeft)/m_encoderWheelsDistance_mm, m_commandManager.getAngleGoal(), m_angleRegulator.getError());
            }

            m_asservCounter = 0;
        }

        /*
         * Regulation en vitesse
         */
        if (m_asservMode == normal_mode || m_asservMode == regulator_output_control) {
            // Mise à jour des consignes en vitesse avec acceleration limitée
            m_speedControllerRight.setSpeedGoal(m_distSpeedLimited + m_angleSpeedLimited);
            m_speedControllerLeft.setSpeedGoal(m_distSpeedLimited - m_angleSpeedLimited);
        } else {
            /* Ici, on ajoute un mode de fonctionnement pour pouvoir controler indépendamment les roues avec l'IHM ou le shell.
             * C'est batard, et cela ne doit pas être utilisé autrement que pour faire du réglage
             */
            m_speedControllerRight.setSpeedGoal(m_directSpeedMode_rightWheelSpeed);
            m_speedControllerLeft.setSpeedGoal(m_directSpeedMode_leftWheelSpeed);
        }

        float outputSpeedRight = m_speedControllerRight.update(estimatedSpeedRight);
        float outputSpeedLeft = m_speedControllerLeft.update(estimatedSpeedLeft);

        if (m_enableMotors) {
            m_motorController.setMotorRightSpeed(outputSpeedRight);
            m_motorController.setMotorLeftSpeed(outputSpeedLeft);
        }

        if( m_blockingDetector )
        {
            m_blockingDetector->update();
        }

        SampleStream *instance = SampleStream::instance();

        instance->setSpeedEstimatedRight(estimatedSpeedRight);
        instance->setSpeedEstimatedLeft(estimatedSpeedLeft);
        instance->setSpeedGoalRight(m_speedControllerRight.getSpeedGoal());
        instance->setSpeedGoalLeft(m_speedControllerLeft.getSpeedGoal());
        instance->setSpeedOutputRight(outputSpeedRight);
        instance->setSpeedOutputLeft(outputSpeedLeft);
        instance->setSpeedIntegratedOutputRight(m_speedControllerRight.getIntegratedOutput());
        instance->setSpeedIntegratedOutputLeft(m_speedControllerLeft.getIntegratedOutput());
        instance->setSpeedKpRight(m_speedControllerRight.getCurrentKp());
        instance->setSpeedKpLeft(m_speedControllerLeft.getCurrentKp());
        instance->setSpeedKiRight(m_speedControllerRight.getCurrentKi());
        instance->setSpeedKiLeft(m_speedControllerLeft.getCurrentKi());


        instance->setAngleGoal(m_commandManager.getAngleGoal());
        instance->setAngleAccumulator(m_angleRegulator.getAccumulator());
        instance->setAngleOutput(m_angleRegulatorOutputSpeedConsign);
        instance->setAngleOutputLimited(m_angleSpeedLimited);

        instance->setDistGoal(m_commandManager.getDistanceGoal());
        instance->setDistAccumulator(m_distanceRegulator.getAccumulator());
        instance->setDistOutput(m_distRegulatorOutputSpeedConsign);
        instance->setDistOutputLimited(m_distSpeedLimited);

        instance->setOdoX(m_odometry.getX());
        instance->setOdoY(m_odometry.getY());
        instance->setOdoTheta(m_odometry.getTheta());


        instance->setRawEncoderDeltaLeft(encoderDeltaLeft);
        instance->setRawEncoderDeltaRight(encoderDeltaRight);

        instance->sendCurrentStream();

        m_asservCounter++;

        chDbgAssert(chVTGetSystemTime() < time, "asserv thread missed deadline !");
        chThdSleepUntil(time);
        time += TIME_US2I(loopPeriod_us);
    }
}

void AsservMain::setRegulatorsSpeed(float distSpeed, float angleSpeed)
{
    chSysLock();
    m_asservMode = regulator_output_control;
    m_distSpeedLimited = distSpeed;
    m_angleSpeedLimited = (angleSpeed * m_encoderWheelsDistance_mm) * 0.5;

    chSysUnlock();
}

void AsservMain::setWheelsSpeed(float rightWheelSpeed, float leftWheelSpeed)
{
    chSysLock();
    m_asservMode = direct_speed_mode;
    m_directSpeedMode_rightWheelSpeed = rightWheelSpeed;
    m_directSpeedMode_leftWheelSpeed = leftWheelSpeed;
    m_distanceRegulatorAccelerationLimiter.reset();
    m_angleRegulatorAccelerationLimiter.reset();
    chSysUnlock();
}

void AsservMain::resetToNormalMode()
{
    chSysLock();
    if (m_asservMode != normal_mode) {
        m_asservMode = normal_mode;
        m_distanceRegulatorAccelerationLimiter.reset();
        m_angleRegulatorAccelerationLimiter.reset();
    }
    chSysUnlock();
}

void AsservMain::enableMotors(bool enable)
{
    chSysLock();
    m_enableMotors = enable;
    if (enable)
    {
        // Ici, il faut reset les intégrateurs des asserv en vitesse
        m_speedControllerLeft.resetIntegral();
        m_speedControllerRight.resetIntegral();
    }
    chSysUnlock();

    if (!enable)
    {
        m_motorController.setMotorRightSpeed(0);
        m_motorController.setMotorLeftSpeed(0);
    }
}

void AsservMain::setEmergencyStop()
{
    chSysLock();
    m_commandManager.setEmergencyStop();
    m_angleRegulatorAccelerationLimiter.disable();
    m_distanceRegulatorAccelerationLimiter.disable();
    m_angleRegulator.disable();
    m_distanceRegulator.disable();
    chSysUnlock();
}

void AsservMain::resetEmergencyStop()
{
    chSysLock();
    m_commandManager.resetEmergencyStop();
    m_angleRegulatorAccelerationLimiter.enable();
    m_distanceRegulatorAccelerationLimiter.enable();
    m_angleRegulator.enable();
    m_distanceRegulator.enable();
    chSysUnlock();
}

void AsservMain::enableAngleRegulator()
{
    chSysLock();
    m_angleRegulator.enable();
    chSysUnlock();
}

void AsservMain::disableAngleRegulator()
{
    chSysLock();
    m_angleRegulator.disable();
    chSysUnlock();
}

void AsservMain::enableDistanceRegulator()
{
    chSysLock();
    m_distanceRegulator.enable();
    chSysUnlock();
}

void AsservMain::disableDistanceRegulator()
{
    chSysLock();
    m_distanceRegulator.disable();
    chSysUnlock();
}

void AsservMain::setPosition(float X_mm, float Y_mm, float theta_rad)
{
    chSysLock();
    m_odometry.setPosition(X_mm, Y_mm, theta_rad);
    /* CommandManager shall be reseted,
     *    because when the current position is overrided in the odometry,
     *    enqued consign in the commandManager are false
     */
    m_commandManager.reset();
//    m_angleRegulator.reset();
//    m_distanceRegulator.reset();
//    m_speedControllerRight.resetIntegral();
//    m_speedControllerLeft.resetIntegral();
//    m_angleRegulatorAccelerationLimiter.reset();
//    m_distanceRegulatorAccelerationLimiter.reset();
    chSysUnlock();
}

void AsservMain::limitMotorControllerConsignToPercentage(float percentage)
{
    /*
     * Use this method to make an low speed for your robot
     */
    chDbgAssert(percentage >= 0 && percentage <= 100, "Percentage shall be in [0;100]");
    chSysLock();
    m_speedControllerLeft.setMaxOutputLimit(percentage);
    m_speedControllerRight.setMaxOutputLimit(percentage);
//    m_speedControllerRight.resetIntegral();
//    m_speedControllerLeft.resetIntegral();
    chSysUnlock();
}

void AsservMain::enablePolar(bool enable)
{
    chSysLock();
    m_enablePolar = enable;
    chSysUnlock();
}

void AsservMain::reset()
{
    /* Pour etre certain de conserver la cohérence des infos,
     * le reset est en section critique
     */
    chSysLock();
    m_odometry.reset();
    m_speedControllerRight.resetIntegral();
    m_speedControllerLeft.resetIntegral();
    m_angleRegulator.reset();
    m_distanceRegulator.reset();
    m_angleRegulatorAccelerationLimiter.reset();
    m_distanceRegulatorAccelerationLimiter.reset();
    m_commandManager.reset();
    m_pllRight.reset();
    m_pllLeft.reset();
    chSysUnlock();
}



