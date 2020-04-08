#include "AsservMain.h"
#include "ch.h"
#include "hal.h"
#include "commandManager/CommandManager.h"
#include "USBStream.h"
#include "Odometry.h"
#include "SlopeFilter.h"
#include "Pll.h"
#include <chprintf.h>
#include "Encoders/Encoder.h"
#include "util/asservMath.h"

AsservMain::AsservMain(uint16_t loopFrequency, uint16_t speedPositionLoopDivisor, float wheelRadius_mm,
        float encoderWheelsDistance_mm, float encodersTicksByTurn, CommandManager &commandManager,
        MotorController &motorController, Encoders &encoders, Odometry &odometrie, Regulator &angleRegulator,
        Regulator &distanceRegulator, SlopeFilter &angleRegulatorSlopeFilter, SlopeFilter &distanceRegulatorSlopeFilter,
        SpeedController &speedControllerRight, SpeedController &speedControllerLeft, Pll &rightPll, Pll &leftPll) :

        m_motorController(motorController), m_encoders(encoders), m_odometry(odometrie), m_speedControllerRight(
                speedControllerRight), m_speedControllerLeft(speedControllerLeft), m_angleRegulator(angleRegulator), m_distanceRegulator(
                distanceRegulator), m_angleRegulatorSlopeFilter(angleRegulatorSlopeFilter), m_distanceRegulatorSlopeFilter(
                distanceRegulatorSlopeFilter), m_commandManager(commandManager), m_pllRight(rightPll), m_pllLeft(
                leftPll), m_distanceByEncoderTurn_mm(M_2PI * wheelRadius_mm), m_encodersTicksByTurn(
                encodersTicksByTurn), m_encodermmByTicks(m_distanceByEncoderTurn_mm / m_encodersTicksByTurn), m_encoderWheelsDistance_mm(
                encoderWheelsDistance_mm), m_encoderWheelsDistance_ticks(encoderWheelsDistance_mm / m_encodermmByTicks), m_loopFrequency(
                loopFrequency), m_loopPeriod(1.0 / float(loopFrequency)), m_speedPositionLoopDivisor(
                speedPositionLoopDivisor)
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
    m_motorController.setMotorRightSpeed(0);
    m_motorController.setMotorLeftSpeed(0);

    const time_conv_t loopPeriod_ms = (m_loopPeriod * 1000.0);
    systime_t time = chVTGetSystemTime();
    time += TIME_MS2I(loopPeriod_ms);
    while (true) {
        int16_t encoderDeltaRight;
        int16_t encoderDeltaLeft;
        m_encoders.getValues(&encoderDeltaRight, &encoderDeltaLeft);

        // Mise à jour de la position en polaire
        m_odometry.refresh(encoderDeltaRight * m_encodermmByTicks, encoderDeltaLeft * m_encodermmByTicks);

        // Estimation & mise à jour des feedbacks
        float deltaAngle_radian = estimateDeltaAngle(encoderDeltaRight, encoderDeltaLeft);
        float deltaDistance_mm = estimateDeltaDistance(encoderDeltaRight, encoderDeltaLeft);

        m_angleRegulator.updateFeedback(deltaAngle_radian);
        m_distanceRegulator.updateFeedback(deltaDistance_mm);

        /* Calculer une nouvelle consigne de vitesse a chaque  ASSERV_POSITION_DIVISOR tour de boucle
         * L'asserv en vitesse étant commandé par l'asserv en position, on laisse qq'e tours de boucle
         * à l'asserv en vitesse pour atteindre sa consigne.
         */
        if (m_asservCounter == m_speedPositionLoopDivisor && m_enablePolar) {
            m_commandManager.update(m_odometry.getX(), m_odometry.getY(), m_odometry.getTheta());

            if (m_asservMode == normal_mode) {
                m_angleRegulatorOutputSpeedConsign = m_angleRegulator.updateOutput(m_commandManager.getAngleGoal());
                m_distRegulatorOutputSpeedConsign = m_distanceRegulator.updateOutput(
                        m_commandManager.getDistanceGoal());
            }

            m_asservCounter = 0;
        }

        /*
         * Regulation en vitesse
         */
        m_pllRight.update(encoderDeltaRight, m_loopPeriod);
        float estimatedSpeedRight = convertSpeedTommSec(m_pllRight.getSpeed());

        m_pllLeft.update(encoderDeltaLeft, m_loopPeriod);
        float estimatedSpeedLeft = convertSpeedTommSec(m_pllLeft.getSpeed());

        if (m_asservMode == normal_mode || m_asservMode == regulator_output_control) {
            // On limite l'acceleration sur la sortie du regulateur de distance et d'angle
            m_distSpeedLimited = m_distanceRegulatorSlopeFilter.filter(m_loopPeriod,
                    m_distRegulatorOutputSpeedConsign);
            m_angleSpeedLimited = m_angleRegulatorSlopeFilter.filter(m_loopPeriod,
                    m_angleRegulatorOutputSpeedConsign);

            // Mise à jour des consignes en vitesse avec acceleration limitée
            m_speedControllerRight.setSpeedGoal(m_distSpeedLimited + m_angleSpeedLimited);
            m_speedControllerLeft.setSpeedGoal(m_distSpeedLimited - m_angleSpeedLimited);
        } else {
            /* Ici, on ajoute un mode de fonctionnement pour pouvoir controler indépendamment les roues avec l'IHM ou le shell.
             * C'est batard, et cela ne doit pas être utilisé autrement que pour faire du réglage
             *      on réutilise les filtres de pente pour ne pas avoir à en instancier d'autres
             */
            float rightWheelSpeed = m_distanceRegulatorSlopeFilter.filter(m_loopPeriod,
                    m_directSpeedMode_rightWheelSpeed);
            float leftWheelSpeed = m_angleRegulatorSlopeFilter.filter(m_loopPeriod,
                    m_directSpeedMode_leftWheelSpeed);
            m_speedControllerRight.setSpeedGoal(rightWheelSpeed);
            m_speedControllerLeft.setSpeedGoal(leftWheelSpeed);
        }

        float outputSpeedRight = m_speedControllerRight.update(estimatedSpeedRight);
        float outputSpeedLeft = m_speedControllerLeft.update(estimatedSpeedLeft);

        if (m_enableMotors) {
            m_motorController.setMotorRightSpeed(outputSpeedRight);
            m_motorController.setMotorLeftSpeed(outputSpeedLeft);
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

        USBStream::instance()->setDistGoal(m_commandManager.getDistanceGoal());
        USBStream::instance()->setDistAccumulator(m_distanceRegulator.getAccumulator());
        USBStream::instance()->setDistOutput(m_distRegulatorOutputSpeedConsign);
        USBStream::instance()->setDistOutputLimited(m_distSpeedLimited);

        USBStream::instance()->setOdoX(m_odometry.getX());
        USBStream::instance()->setOdoY(m_odometry.getY());
        USBStream::instance()->setOdoTheta(m_odometry.getTheta());

        USBStream::instance()->setRawEncoderDeltaLeft((float) encoderDeltaLeft);
        USBStream::instance()->setRawEncoderDeltaRight((float) encoderDeltaRight);

        USBStream::instance()->SendCurrentStream();

        m_asservCounter++;

        chDbgAssert(chVTGetSystemTime() < time, "asserv thread missed deadline !");
        chThdSleepUntil(time);
        time += TIME_MS2I(loopPeriod_ms);
    }
}

void AsservMain::setRegulatorsSpeed(float distSpeed, float angleSpeed)
{
    m_asservMode = regulator_output_control;
    m_distRegulatorOutputSpeedConsign = distSpeed;
    m_angleRegulatorOutputSpeedConsign = angleSpeed * m_encoderWheelsDistance_mm;
}

void AsservMain::setWheelsSpeed(float rightWheelSpeed, float leftWheelSpeed)
{
    m_asservMode = direct_speed_mode;
    m_directSpeedMode_rightWheelSpeed = rightWheelSpeed;
    m_directSpeedMode_leftWheelSpeed = leftWheelSpeed;
    m_distanceRegulatorSlopeFilter.reset();
    m_angleRegulatorSlopeFilter.reset();
}

void AsservMain::resetToNormalMode()
{
    if (m_asservMode != normal_mode) {
        m_asservMode = normal_mode;
        m_distanceRegulatorSlopeFilter.reset();
        m_angleRegulatorSlopeFilter.reset();
    }
}

void AsservMain::enableMotors(bool enable)
{
    m_enableMotors = enable;
    if (enable) {
        // Ici, il faut reset les intégrateurs des asserv en vitesse
        m_speedControllerLeft.resetIntegral();
        m_speedControllerRight.resetIntegral();
    }
    else {
        m_motorController.setMotorRightSpeed(0);
        m_motorController.setMotorLeftSpeed(0);
    }
}

void AsservMain::enablePolar(bool enable)
{
    m_enablePolar = enable;
}
