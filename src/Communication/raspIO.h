
#ifndef SRC_ROBOTS_PRINCESS_RASPIO_H_
#define SRC_ROBOTS_PRINCESS_RASPIO_H_

#include "ch.h"

class Odometry;
class CommandManager;
class MotorController;
class mainAsserv;


class RaspIO
{
public:

    explicit RaspIO(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController,
        AsservMain &mainAsserv);
    virtual ~RaspIO() {};

    void positionOutput();
    void commandInput();



private:
    SerialDriver *m_serialDriver;
    BaseSequentialStream *m_outputStream;
    Odometry &m_odometry;
    CommandManager &m_commandManager;
    MotorController &m_motorController;
    AsservMain &m_mainAsserv;


    static void serialReadLine(SerialDriver *serialDriver, char *buffer, unsigned int buffer_size);
};


#endif /* SRC_ROBOTS_PRINCESS_RASPIO_H_ */
