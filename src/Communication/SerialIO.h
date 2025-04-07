
#ifndef SRC_COMMUNICATION_SERIALIO_H_
#define SRC_COMMUNICATION_SERIALIO_H_

#include "ch.h"

class Odometry;
class CommandManager;
class MotorController;
class mainAsserv;


class SerialIO
{
public:

    explicit SerialIO(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController,
        AsservMain &mainAsserv);
    virtual ~SerialIO() {};

    void positionOutput();
    virtual void commandInput();



protected:
    SerialDriver *m_serialDriver;
    BaseSequentialStream *m_outputStream;
    Odometry &m_odometry;
    CommandManager &m_commandManager;
    MotorController &m_motorController;
    AsservMain &m_mainAsserv;

    static void serialReadLine(SerialDriver *serialDriver, char *buffer, unsigned int buffer_size);
    bool classicCommandHandle(char readChar);
};


#endif /* SRC_COMMUNICATION_SERIALIO_H_ */
