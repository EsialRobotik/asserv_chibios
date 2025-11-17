#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cfloat>
#include "util/asservMath.h"
#include "Odometry.h"
#include "AsservMain.h"
#include "SerialCbor.h"
#include "commandManager/CommandManager.h"
#include "motorController/MotorController.h"
#include "AsservMain.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "qcbor/qcbor_encode.h"
#include "Crc/CrcCalculator.h"


const uint32_t synchroWord = 0xDEADBEEF;


SerialCbor::SerialCbor(SerialDriver *serialDriver, Crc32Calculator *crc32Calculator,
    Odometry &odometry, CommandManager &commandManager, MotorController &motorController, AsservMain &mainAsserv,
    SimpleAccelerationLimiter *angleAccelerationlimiter, AccelerationDecelerationLimiter *distanceAccelerationLimiter,
    AccDecConfiguration *normalAccDec, AccDecConfiguration *slowAccDec)
: m_cborSm(crc32Calculator), m_odometry(odometry), m_commandManager(commandManager), m_motorController(motorController), m_mainAsserv(mainAsserv)
{   
    m_serialDriver = serialDriver;
    m_outputStream = reinterpret_cast<BaseSequentialStream*>(serialDriver);
    m_crc32Calculator = crc32Calculator;

    chDbgAssert( 
            (angleAccelerationlimiter != nullptr && distanceAccelerationLimiter != nullptr && normalAccDec != nullptr &&  slowAccDec != nullptr)
         || (angleAccelerationlimiter == nullptr && distanceAccelerationLimiter == nullptr && normalAccDec == nullptr &&  slowAccDec == nullptr),
         "angleAccelerationlimiter, distanceAccelerationLimiter, normalAccDec and slowAccDec shall all be null or none of them");


    m_angleAccelerationlimiter = angleAccelerationlimiter;
    m_distanceAccelerationLimiter = distanceAccelerationLimiter;
    if( normalAccDec )
        m_normalAccDec = *normalAccDec;
    if( slowAccDec)
        m_slowAccDec = *slowAccDec;

}


void SerialCbor::positionOutput()
{
    uint32_t *syncWordPtr = (uint32_t*)m_qcborOutputBuffer;
    syncWordPtr[0] = synchroWord;
    uint32_t *crcWordPtr =  &syncWordPtr[1];
    uint32_t *encodedSizeWordPtr =  &syncWordPtr[2];

    QCBOREncodeContext EncodeCtx;
    UsefulBuf qcoborBuffer = {m_qcborOutputBuffer+4*3, sizeof(m_qcborOutputBuffer)};

    int idx=0;

    const time_conv_t loopPeriod_ms = 100;
    systime_t time = chVTGetSystemTime();
    time += TIME_MS2I(loopPeriod_ms);
    while(true)
    {
        QCBOREncode_Init(&EncodeCtx, qcoborBuffer);

        // QCBOREncode_OpenMap(&EncodeCtx);
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "idx", m_commandManager.getCurrentCommandIndex());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "X", (int32_t)m_odometry.getX());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "Y", (int32_t)m_odometry.getY());
        // QCBOREncode_AddFloatToMapSZ(&EncodeCtx, "theta", m_odometry.getTheta());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "stat", m_commandManager.getCommandStatus());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "cnt", m_commandManager.getPendingCommandCount());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "ML", (int8_t)m_motorController.getMotorLeftSpeedNonInverted());
        // QCBOREncode_AddInt64ToMapSZ(&EncodeCtx, "MR", (int8_t)m_motorController.getMotorRightSpeedNonInverted());
        // QCBOREncode_CloseMap(&EncodeCtx);

        QCBOREncode_OpenArray(&EncodeCtx);
        QCBOREncode_AddInt64(&EncodeCtx, (int32_t)m_odometry.getX());
        QCBOREncode_AddInt64(&EncodeCtx, (int32_t)m_odometry.getY());
        QCBOREncode_AddFloat(&EncodeCtx, m_odometry.getTheta());
        QCBOREncode_AddInt64(&EncodeCtx, m_commandManager.getCurrentCommandIndex());
        QCBOREncode_AddInt64(&EncodeCtx, m_commandManager.getCommandStatus());
        QCBOREncode_AddInt64(&EncodeCtx, m_commandManager.getPendingCommandCount());
        QCBOREncode_AddInt64(&EncodeCtx, (int8_t)m_motorController.getMotorLeftSpeedNonInverted());
        QCBOREncode_AddInt64(&EncodeCtx, (int8_t)m_motorController.getMotorRightSpeedNonInverted());
        QCBOREncode_CloseArray(&EncodeCtx);


        
        UsefulBufC EncodedCBOR;
        QCBORError uErr;
        uErr = QCBOREncode_Finish(&EncodeCtx, &EncodedCBOR);
        if(uErr == QCBOR_SUCCESS) 
        {
            *encodedSizeWordPtr = EncodedCBOR.len;
            *crcWordPtr = m_crc32Calculator->compute(EncodedCBOR.ptr, EncodedCBOR.len);
            // crcAcquireUnit(&CRCD1);
            // crcReset(&CRCD1);
            //  = crcCalc(&CRCD1, , );
            // crcReleaseUnit(&CRCD1);


            streamWrite(m_serialDriver, (const uint8_t*)m_qcborOutputBuffer, EncodedCBOR.len+4*3);
        } 
        chThdSleepUntil(time);
        time += TIME_MS2I(loopPeriod_ms);
    }
}


void SerialCbor::commandInput()
{
    while(true)
    {
        int read = sdReadTimeout(m_serialDriver, m_qcborInputBuffer, sizeof(m_qcborInputBuffer), TIME_MS2I(10));
        for(int i=0; i<read; i++)
            m_cborSm.push_byte(m_qcborInputBuffer[i]);

        CborStreamStateMachine::cmd_t cmd;
        if( m_cborSm.get_cmd(cmd)) 
           decode_cmd(cmd);
    }
}


void SerialCbor::decode_cmd(CborStreamStateMachine::cmd_t &cmd)
{
    switch (cmd.cmd_type) 
    {
        case CborStreamStateMachine::emergency_stop :
            m_mainAsserv.setEmergencyStop();
        break;

        case CborStreamStateMachine::emergency_stop_reset:
            m_mainAsserv.resetEmergencyStop();
        break;

        case CborStreamStateMachine::turn:
            m_commandManager.addTurn(degToRad(cmd.arg1), cmd.cmd_id);
        break;

        case CborStreamStateMachine::straight:
            m_commandManager.addStraightLine(cmd.arg1, cmd.cmd_id);
        break;

        case CborStreamStateMachine::face:
            m_commandManager.addGoToAngle(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::goto_front:
            m_commandManager.addGoTo(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::goto_back:
            m_commandManager.addGoToBack(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::goto_nostop:
            m_commandManager.addGoToNoStop(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::orbital_turn:
            // Use float as boolean is absolutely not a thing todo, but this case doesn't exist when this was conceived. TODO: refactor to have a clear design!
            m_commandManager.addGOrbitalTurn(degToRad(cmd.arg1), (cmd.arg2 == 1.0), (cmd.arg3 == 1.0), cmd.cmd_id);
        break;

        case CborStreamStateMachine::max_motor_speed:
            m_mainAsserv.limitMotorControllerConsignToPercentage(cmd.arg1);
        break;

        case CborStreamStateMachine::normal_speed_acc_mode:
            if( m_angleAccelerationlimiter)
            {
                m_angleAccelerationlimiter->setMaxAcceleration(m_normalAccDec.maxAngleAcceleration);
                m_distanceAccelerationLimiter->disable();
                m_distanceAccelerationLimiter->setMaxAccFW(m_normalAccDec.maxDistAccelerationForward);
                m_distanceAccelerationLimiter->setMaxDecFW(m_normalAccDec.maxDistDecelerationForward);
                m_distanceAccelerationLimiter->setMaxAccBW(m_normalAccDec.maxDistAccelerationBackward);
                m_distanceAccelerationLimiter->setMaxDecBW(m_normalAccDec.maxDistDecelerationBackward);
                m_distanceAccelerationLimiter->enable();
            }
        break;

        case CborStreamStateMachine::slow_speed_acc_mode:
            if( m_angleAccelerationlimiter)
            {
                m_angleAccelerationlimiter->setMaxAcceleration(m_slowAccDec.maxAngleAcceleration);
                m_distanceAccelerationLimiter->disable();
                m_distanceAccelerationLimiter->setMaxAccFW(m_slowAccDec.maxDistAccelerationForward);
                m_distanceAccelerationLimiter->setMaxDecFW(m_slowAccDec.maxDistDecelerationForward);
                m_distanceAccelerationLimiter->setMaxAccBW(m_slowAccDec.maxDistAccelerationBackward);
                m_distanceAccelerationLimiter->setMaxDecBW(m_slowAccDec.maxDistDecelerationBackward);
                m_distanceAccelerationLimiter->enable();
            }
        break;
    }
}