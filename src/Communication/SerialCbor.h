
#ifndef SRC_COMMUNICATION_SERIALCBOR_H_
#define SRC_COMMUNICATION_SERIALCBOR_H_

#include "ch.h"
#include "cborStream/CborStreamStateMachine.h"
class Odometry;
class CommandManager;
class MotorController;
class mainAsserv;
class SimpleAccelerationLimiter;
class AccelerationDecelerationLimiter;

class SerialCbor
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

    explicit SerialCbor(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController,
        AsservMain &mainAsserv, SimpleAccelerationLimiter *angleAccelerationlimiter = nullptr, AccelerationDecelerationLimiter *distanceAccelerationLimiter= nullptr,
        AccDecConfiguration *normalAccDec= nullptr, AccDecConfiguration *slowAccDec= nullptr);
    virtual ~SerialCbor() {};

    void positionOutput();

    void commandInput();

private:
    CborStreamStateMachine m_cborSm;
    SerialDriver *m_serialDriver;
    BaseSequentialStream *m_outputStream;
    Odometry &m_odometry;
    CommandManager &m_commandManager;
    MotorController &m_motorController;
    AsservMain &m_mainAsserv;
    SimpleAccelerationLimiter *m_angleAccelerationlimiter;
    AccelerationDecelerationLimiter *m_distanceAccelerationLimiter;
    AccDecConfiguration m_normalAccDec;
    AccDecConfiguration m_slowAccDec;
    uint8_t m_qcborOutputBuffer[64]; 
    uint8_t m_qcborInputBuffer[32]; 

    void decode_cmd(CborStreamStateMachine::cmd_t &cmd);
};


#endif /* SRC_COMMUNICATION_SERIALCBOR_H_ */
