#ifndef ASSERVMAIN_H_
#define ASSERVMAIN_H_

#include "motorController/MotorController.h"
#include <cstdint>

class CommandManager;
class Encoders;
class Odometry;
class Pll;
class AccelerationDecelerationLimiterInterface;
class SpeedController;
class Regulator;
class BlockingDetector;


class AsservMain
{
public:


    typedef enum {
        mixing_type_polar,
        mixing_type_angle_regulator_right_wheel_only,
        mixing_type_angle_regulator_left_wheel_inverted_only,
        mixing_type_direct_speed
    } mixing_type_t;


    explicit AsservMain(uint16_t loopFrequency, uint16_t speedPositionLoopDivisor, float wheelRadius_mm,
            float encoderWheelsDistance_mm, uint32_t encodersTicksByTurn, CommandManager &commandManager,
            MotorController &motorController, Encoders &encoders, Odometry &odometrie,
            Regulator &angleRegulator, Regulator &distanceRegulator,
            AccelerationDecelerationLimiterInterface &angleRegulatorAccelerationLimiter, AccelerationDecelerationLimiterInterface &distanceRegulatorAccelerationLimiter,
            SpeedController &speedControllerRight, SpeedController &speedControllerLeft,
            Pll &rightPll, Pll &leftPll,
            BlockingDetector *blockingDetector = nullptr);

    virtual ~AsservMain()
    {
    }
    ;

    void mainLoop();

    void enableMotors(bool enable);

    void reset();

    void setEmergencyStop();
    void resetEmergencyStop();

    void enableAngleRegulator();
    void disableAngleRegulator();
    void enableDistanceRegulator();
    void disableDistanceRegulator();

    void setPosition(float X_mm, float Y_mm, float theta_rad);
    void limitMotorControllerConsignToPercentage(float percentage);

    void setEncodersWheelsDistance_mm(float wheelsDistance_mm);
private:

    float convertSpeedTommSec(float speed_ticksPerSec);
    float estimateDeltaAngle(int16_t deltaCountRight, int16_t deltaCountLeft);
    float estimateDeltaDistance(int16_t deltaCountRight, int16_t deltaCountLeft);

    typedef enum
    {
        normal_mode, direct_speed_mode, regulator_output_control
    } asserv_mode_t;

    MotorController &m_motorController;
    Encoders &m_encoders;
    Odometry &m_odometry;
    SpeedController &m_speedControllerRight;
    SpeedController &m_speedControllerLeft;
    Regulator &m_angleRegulator;
    Regulator &m_distanceRegulator;
    AccelerationDecelerationLimiterInterface &m_angleRegulatorAccelerationLimiter;
    AccelerationDecelerationLimiterInterface &m_distanceRegulatorAccelerationLimiter;
    CommandManager &m_commandManager;
    Pll &m_pllRight;
    Pll &m_pllLeft;
    BlockingDetector *m_blockingDetector;

    const float m_distanceByEncoderTurn_mm;
    const float m_encodersTicksByTurn;
    const float m_encodermmByTicks;
    float m_encoderWheelsDistance_mm;
    float m_encoderWheelsDistance_ticks;

    const uint16_t m_loopFrequency;
    const float m_loopPeriod;
    const uint16_t m_speedPositionLoopDivisor;
    uint8_t m_asservCounter;

    float m_distRegulatorOutputSpeedConsign;
    float m_distSpeedLimited;

    float m_angleRegulatorOutputSpeedConsign;
    float m_angleSpeedLimited;

    bool m_enableMotors;
};

#endif /* ASSERVMAIN_H_ */
