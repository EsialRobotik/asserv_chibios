
#ifndef SRC_COMMUNICATION_RASPIO_H_
#define SRC_COMMUNICATION_RASPIO_H_

#include "ch.h"
#include "SerialIO.h"

class SimpleAccelerationLimiter;
class AccelerationDecelerationLimiter;

class RaspIO : SerialIO
{
public:

    struct AccDecConfiguration
    {
        float maxAngleAcceleration;
        float maxDistAccelerationForward;
        float maxDistDecelerationForward;
        float maxDistAccelerationBackward;
        float maxDistDecelerationBackward;
    };

    explicit RaspIO(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController,
        AsservMain &mainAsserv, SimpleAccelerationLimiter *angleAccelerationlimiter = nullptr, AccelerationDecelerationLimiter *distanceAccelerationLimiter = nullptr,
        AccDecConfiguration *normalAccDec = nullptr, AccDecConfiguration *slowAccDec = nullptr);
    virtual ~RaspIO() {};

    virtual void commandInput();

private:
    SimpleAccelerationLimiter *m_angleAccelerationlimiter;
    AccelerationDecelerationLimiter *m_distanceAccelerationLimiter;
    AccDecConfiguration m_normalAccDec;
    AccDecConfiguration m_slowAccDec;

    
    bool customCommandHandle(char readChar);
};


#endif /* SRC_COMMUNICATION_RASPIO_H_ */
