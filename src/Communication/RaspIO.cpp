#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cfloat>
#include "util/asservMath.h"
#include "config.h"
#include "Odometry.h"
#include "RaspIO.h"
#include "commandManager/CommandManager.h"
#include "motorController/MotorController.h"
#include "AsservMain.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"



RaspIO::RaspIO(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController,
    AsservMain &mainAsserv, SimpleAccelerationLimiter &angleAccelerationlimiter, AccelerationDecelerationLimiter &distanceAccelerationLimiter,
    AccDecConfiguration &normalAccDec, AccDecConfiguration &slowAccDec)
: SerialIO(serialDriver, odometry, commandManager, motorController, mainAsserv), m_angleAccelerationlimiter(angleAccelerationlimiter), m_distanceAccelerationLimiter(distanceAccelerationLimiter)
{   
    m_normalAccDec = normalAccDec;
    m_slowAccDec = slowAccDec;
}

bool RaspIO::customCommandHandle(char readChar)
{
    bool res = true;
    switch (readChar) 
    {

        case 'N': // Restore du comportement Normal du robot
            m_angleAccelerationlimiter.setMaxAcceleration(m_normalAccDec.maxAngleAcceleration);
        	m_distanceAccelerationLimiter.disable();
        	m_distanceAccelerationLimiter.setMaxAccFW(m_normalAccDec.maxDistAccelerationForward);
        	m_distanceAccelerationLimiter.setMaxDecFW(m_normalAccDec.maxDistDecelerationForward);
        	m_distanceAccelerationLimiter.setMaxAccBW(m_normalAccDec.maxDistAccelerationBackward);
        	m_distanceAccelerationLimiter.setMaxDecBW(m_normalAccDec.maxDistDecelerationBackward);
        	m_distanceAccelerationLimiter.enable();
            break;

		case 'n' : // On réduit l'acceleration/décélleration en cas d'objets dans les pinces
            m_angleAccelerationlimiter.setMaxAcceleration(m_slowAccDec.maxAngleAcceleration);
        	m_distanceAccelerationLimiter.disable();
        	m_distanceAccelerationLimiter.setMaxAccFW(m_slowAccDec.maxDistAccelerationForward);
        	m_distanceAccelerationLimiter.setMaxDecFW(m_slowAccDec.maxDistDecelerationForward);
        	m_distanceAccelerationLimiter.setMaxAccBW(m_slowAccDec.maxDistAccelerationBackward);
        	m_distanceAccelerationLimiter.setMaxDecBW(m_slowAccDec.maxDistDecelerationBackward);
        	m_distanceAccelerationLimiter.enable();
			break;

        default:
            
            res = false;
            break;            
    }
    return res;
}


void RaspIO::commandInput()
{

    chprintf(m_outputStream, "Started\r\n");


    while(true)
    {
        char readChar = streamGet(m_serialDriver);
        bool res = classicCommandHandle(readChar);

        if( !res )
        {
            res = customCommandHandle(readChar);
        }

        if( !res )
        {
            chprintf(m_outputStream, " - unexpected character\r\n");
        }
    }
}
