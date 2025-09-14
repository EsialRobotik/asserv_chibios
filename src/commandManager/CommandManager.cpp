#include "CommandManager.h"
#include "Commands/StraitLine.h"
#include "Commands/Turn.h"
#include "Commands/OrbitalTurn.h"
#include "Commands/Goto.h"
#include "Commands/GotoAngle.h"
#include "Commands/WheelSpeed.h"
#include "blockingDetector/BlockingDetector.h"
#include <cstdlib>
#include <cmath>
#include <new>
#include "util/asservMath.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>


#define MAX(a,b) (((a)>(b))?(a):(b))
#define COMMAND_MAX_SIZE MAX( MAX( MAX( MAX( MAX( MAX( sizeof(WheelSpeed), sizeof(OrbitalTurn)), sizeof(StraitLine)), sizeof(Turn)), sizeof(Goto)), sizeof(GotoAngle) ), sizeof(GotoNoStop) )

CommandManager::CommandManager(float straitLineArrivalWindows_mm, float turnArrivalWindows_rad,
        Goto::GotoConfiguration &preciseGotoConfiguration, Goto::GotoConfiguration &waypointGotoConfiguration, GotoNoStop::GotoNoStopConfiguration &gotoNoStopConfiguration,
        Regulator &angle_regulator, Regulator &distance_regulator,
        float angle_regulator_normal_max_output, float angle_regulator_orbital_max_output,
        AccelerationDecelerationLimiter *accelerationDecelerationLimiter,
        BlockingDetector *blockingDetector):
        m_cmdList(32,COMMAND_MAX_SIZE),
        m_straitLineArrivalWindows_mm(straitLineArrivalWindows_mm), m_turnArrivalWindows_rad(turnArrivalWindows_rad),
        m_preciseGotoConfiguration(preciseGotoConfiguration), m_waypointGotoConfiguration(waypointGotoConfiguration), m_gotoNoStopConfiguration(gotoNoStopConfiguration),
        m_angle_regulator(angle_regulator), m_distance_regulator(distance_regulator)
{
    m_emergencyStop = false;
    m_blockingDetected = false;
    m_currentCmd = nullptr;
    m_accelerationDecelerationLimiter = accelerationDecelerationLimiter;
    m_blockingDetector = blockingDetector;
    m_consign.type = Command::consign_type_t::consign_acceleration_limited;
    m_consign.angle_consign = 0;
    m_consign.distance_consign = 0;
    m_angle_regulator_normal_max_output = angle_regulator_normal_max_output;
    m_angle_regulator_orbital_max_output = angle_regulator_orbital_max_output;

}

bool CommandManager::addStraightLine(float valueInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) StraitLine(valueInmm, m_straitLineArrivalWindows_mm);
    ptr->setIndex(index);

    return m_cmdList.push();
}

bool CommandManager::addTurn(float angleInRad, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Turn(angleInRad, m_turnArrivalWindows_rad);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGOrbitalTurn(float angleInRad, bool forward, bool turnToTheRight, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) OrbitalTurn(angleInRad, forward, turnToTheRight, m_turnArrivalWindows_rad, m_angle_regulator_normal_max_output, m_angle_regulator_orbital_max_output, m_angle_regulator);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addWheelsSpeed(float rightWheelSpeedInmmpersec, float leftWheelSpeedInmmpersec, uint32_t stepDurationInms, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
     if(ptr == nullptr)
         return false;

     new (ptr) WheelSpeed(rightWheelSpeedInmmpersec, leftWheelSpeedInmmpersec, stepDurationInms);
     ptr->setIndex(index);

     m_cmdList.push();
     return true;
}


bool CommandManager::addGoTo(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Goto(posXInmm, posYInmm, &m_preciseGotoConfiguration);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToWaypoint(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Goto(posXInmm, posYInmm, &m_waypointGotoConfiguration);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToBack(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) Goto(posXInmm, posYInmm, &m_preciseGotoConfiguration, true);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToNoStop(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) GotoNoStop(posXInmm, posYInmm, &m_gotoNoStopConfiguration, &m_preciseGotoConfiguration, false, m_accelerationDecelerationLimiter);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToNoStopBack(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) GotoNoStop(posXInmm, posYInmm, &m_gotoNoStopConfiguration, &m_preciseGotoConfiguration, true);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

bool CommandManager::addGoToAngle(float posXInmm, float posYInmm, uint32_t index)
{
    Command *ptr = m_cmdList.getFree();
    if(ptr == nullptr)
        return false;

    new (ptr) GotoAngle(posXInmm, posYInmm, m_turnArrivalWindows_rad);
    ptr->setIndex(index);

    m_cmdList.push();
    return true;
}

void CommandManager::setEmergencyStop()
{
    m_consign.type = Command::consign_type_t::consign_acceleration_limited;
    m_consign.angle_consign = m_angle_regulator.getAccumulator();
    m_consign.distance_consign = m_distance_regulator.getAccumulator();

    m_cmdList.flush();
    m_currentCmd = nullptr;

    m_emergencyStop = true;
    m_angle_regulator.setMaxOutput(m_angle_regulator_normal_max_output);
}

void CommandManager::resetEmergencyStop()
{
    m_emergencyStop = false;
    m_blockingDetected = false;
    if(m_blockingDetector)
        m_blockingDetector->reset();
}

CommandManager::CommandStatus CommandManager::getCommandStatus()
{

    if( m_emergencyStop )
        return STATUS_HALTED;
    else if (m_currentCmd == nullptr)
        return STATUS_IDLE;
    else if(m_blockingDetected )
        return STATUS_BLOCKED;
    else
        return STATUS_RUNNING;
}

uint8_t CommandManager::getPendingCommandCount()
{
    return m_cmdList.size();
}

uint32_t CommandManager::getCurrentCommandIndex()
{
    if( m_currentCmd != nullptr)
        return m_currentCmd->getIndex();

    return 0;
}


void CommandManager::switchToNextCommand()
{
    if (m_currentCmd != nullptr)
       m_cmdList.pop();

    m_currentCmd = m_cmdList.getFirst();
}


AsservMain::mixing_type_t CommandManager::getCurrentCommandMixingType() const
{
    AsservMain::mixing_type_t res = AsservMain::mixing_type_polar;
    if( m_currentCmd )
        res =  m_currentCmd->getMixingType();
    return res;
}


void CommandManager::update(float X_mm, float Y_mm, float theta_rad)
{
    if (m_emergencyStop)
    {
        m_cmdList.flush();
        m_currentCmd = nullptr;
        return;
    }
    else if (m_blockingDetector && m_blockingDetector->isBlocked())
    {
        m_blockingDetected = true;
    }


    if (m_currentCmd != nullptr && !m_currentCmd->isGoalReached(X_mm, Y_mm, theta_rad, m_angle_regulator, m_distance_regulator, m_cmdList.getSecond()))
    {
        m_currentCmd->updateConsign(X_mm, Y_mm, theta_rad, m_consign, m_angle_regulator, m_distance_regulator);
    }
    else
    {
        switchToNextCommand();
        if( m_currentCmd != nullptr )
            m_consign.type = m_currentCmd->computeInitialConsign(X_mm, Y_mm, theta_rad, m_consign, m_angle_regulator, m_distance_regulator);
    }
}




