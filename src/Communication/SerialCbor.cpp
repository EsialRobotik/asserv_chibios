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

const uint32_t synchroWord = 0xDEADBEEF;

static const CRCConfig crc32_config = {
    .poly_size         = 32,
    .poly              = 0xf4acfb13,
    .initial_val       = 0xFFFFFFFF,
    .final_val         = 0xffffffff,
    .reflect_data      = 1,
    .reflect_remainder = 1
   };

SerialCbor::SerialCbor(SerialDriver *serialDriver, Odometry &odometry, CommandManager &commandManager, MotorController &motorController, AsservMain &mainAsserv,
    SimpleAccelerationLimiter *angleAccelerationlimiter, AccelerationDecelerationLimiter *distanceAccelerationLimiter,
    AccDecConfiguration *normalAccDec, AccDecConfiguration *slowAccDec)
:m_cborSm(&CRCD1), m_odometry(odometry), m_commandManager(commandManager), m_motorController(motorController), m_mainAsserv(mainAsserv)
{   
    m_serialDriver = serialDriver;
    m_outputStream = reinterpret_cast<BaseSequentialStream*>(serialDriver);

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

    crcAcquireUnit(&CRCD1);
    crcStart(&CRCD1, &crc32_config);
    crcReleaseUnit(&CRCD1);
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
            crcAcquireUnit(&CRCD1);
            crcReset(&CRCD1);
            *crcWordPtr = crcCalc(&CRCD1, EncodedCBOR.len, EncodedCBOR.ptr);
            crcReleaseUnit(&CRCD1);


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
            //  chprintf(reinterpret_cast<BaseSequentialStream*>(&LPSD1), "goto_front %f %f \r\n", cmd.arg1, cmd.arg2);
            m_commandManager.addGoTo(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::goto_back:
            m_commandManager.addGoToBack(cmd.arg1, cmd.arg2, cmd.cmd_id);
        break;

        case CborStreamStateMachine::goto_nostop:
            m_commandManager.addGoToNoStop(cmd.arg1, cmd.arg2);
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