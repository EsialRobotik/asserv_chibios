#include "CommandManager.h"
#include "Commands/StraitLine.h"
#include "Commands/Turn.h"
#include "Commands/Goto.h"
#include "Commands/GotoAngle.h"
#include <cstdlib>
#include <cmath>
#include <new>
#include "util/asservMath.h"
#include "ch.h"
#include "hal.h"
#include "USBStream.h"
#include <chprintf.h>


#define MAX(a,b) (((a)>(b))?(a):(b))
#define COMMAND_MAX_SIZE MAX( MAX( MAX(sizeof(StraitLine), sizeof(Turn)), sizeof(Goto)), sizeof(GotoAngle) )

CommandManager::CommandManager(float straitLineArrivalWindows_mm, float turnArrivalWindows_rad,
        Goto::GotoConfiguration preciseGotoConfiguration, Goto::GotoConfiguration waypointGotoConfiguration,
        const Regulator &angle_regulator, const Regulator &distance_regulator):
		m_cmdList(32,COMMAND_MAX_SIZE),
		m_straitLineArrivalWindows_mm(straitLineArrivalWindows_mm), m_turnArrivalWindows_rad(turnArrivalWindows_rad),
		m_preciseGotoConfiguration(preciseGotoConfiguration), m_waypointGotoConfiguration(waypointGotoConfiguration),
		m_angle_regulator(angle_regulator), m_distance_regulator(distance_regulator)
{
    m_emergencyStop = false;
    m_currentCmd = nullptr;
    m_currentCmdBuffer = (Command*) malloc( COMMAND_MAX_SIZE );
    m_nextCmd = nullptr;
    m_angleRegulatorConsign = 0;
    m_distRegulatorConsign = 0;
}

extern BaseSequentialStream *outputStream;

bool CommandManager::addStraightLine(float valueInmm)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) StraitLine(valueInmm, m_straitLineArrivalWindows_mm);

    chprintf(outputStream, "new StraitLine to %x (%f , %f)\r\n", ptr, valueInmm, m_straitLineArrivalWindows_mm);
    return m_cmdList.push();
}

bool CommandManager::addTurn(float angleInRad)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Turn(angleInRad, m_turnArrivalWindows_rad);
    m_cmdList.push();
    return true;
}

bool CommandManager::addGoTo(float posXInmm, float posYInmm)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Goto(posXInmm, posYInmm, &m_preciseGotoConfiguration);
    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToBack(float posXInmm, float posYInmm)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Goto(posXInmm, posYInmm, &m_preciseGotoConfiguration, true);
    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToNoStop(float , float )
{
    return true;
}

bool CommandManager::addGoToAngle(float posXInmm, float posYInmm)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) GotoAngle(posXInmm, posYInmm, m_turnArrivalWindows_rad);
    m_cmdList.push();
    return true;
}

void CommandManager::setEmergencyStop()
{
    m_angleRegulatorConsign = m_angle_regulator.getAccumulator();
    m_distRegulatorConsign = m_distance_regulator.getAccumulator();

    m_cmdList.flush();
    m_currentCmd = nullptr;
    m_nextCmd = nullptr;

    m_emergencyStop = true;
}

void CommandManager::resetEmergencyStop()
{
    m_emergencyStop = false;
}

CommandStatus CommandManager::getCommandStatus()
{
    if( m_emergencyStop )
        return STATUS_HALTED;
    else if (m_nextCmd == nullptr)
        return STATUS_IDLE;
    else
        return STATUS_RUNNING;
}

uint8_t CommandManager::getPendingCommandCount()
{
    return m_cmdList.size();
}


void CommandManager::switchToNextCommand()
{
    if (m_nextCmd != nullptr)
    {
       // The next command was already retrieved, use it as current command and pop it.
       m_nextCmd->cloneIn(m_currentCmdBuffer);
       m_currentCmd = m_currentCmdBuffer;
       m_cmdList.pop();
    }
    else
    {
       // No next command retrieved, copy the head element (if any) of the list to the local buffer
       m_currentCmd = m_cmdList.getFull();
       if( m_currentCmd != nullptr)
       {
           m_currentCmd->cloneIn(m_currentCmdBuffer);
           m_currentCmd = m_currentCmdBuffer;
           m_cmdList.pop();
       }
    }
}

void CommandManager::tryToRetrieveNextCommand()
{
    if( m_currentCmd == nullptr)
        return;

    if (m_nextCmd == nullptr)
        m_nextCmd = m_cmdList.getFull();
}
void CommandManager::update(float X_mm, float Y_mm, float theta_rad)
{
    if (m_emergencyStop)
    {
        m_cmdList.flush();
        m_currentCmd = nullptr;
        m_nextCmd = nullptr;
        return;
    }

    if (m_currentCmd != nullptr && !m_currentCmd->isGoalReached(X_mm, Y_mm, theta_rad, m_angle_regulator, m_distance_regulator))
    {
        tryToRetrieveNextCommand();
        m_currentCmd->updateConsign(X_mm, Y_mm, theta_rad, &m_distRegulatorConsign, &m_angleRegulatorConsign, m_angle_regulator, m_distance_regulator);
    }
    else
    {
        switchToNextCommand();
        tryToRetrieveNextCommand();
        if( m_currentCmd != nullptr )
            m_currentCmd->computeInitialConsign(X_mm, Y_mm, theta_rad, &m_distRegulatorConsign, &m_angleRegulatorConsign, m_angle_regulator, m_distance_regulator);
    }
}




