#include <stdio.h>
#include "Regulator.h"
#include "Pll.h"
#include "sampleStream/SampleStreamInterface.h"
#include "robotStub/EncoderSimulation.h"
#include "robotStub/MotorControllerSimulation.h"
#include "Odometry.h"
#include "SpeedController/AdaptativeSpeedController.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "commandManager/CommandManager.h"
#include "AsservMain.h"
#include "NativeStream.h"
#include <float.h>

#define ASSERV_THREAD_FREQUENCY (600)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_TICKS_BY_TURN (1440*4)


#define ANGLE_REGULATOR_KP (1)
#define DIST_REGULATOR_KP (1)
#define ANGLE_REGULATOR_MAX_ACC (3000)
#define MAX_SPEED_MM_PER_SEC (1500)
#define PLL_BANDWIDTH (150)
#define ENCODERS_WHEELS_RADIUS_MM (31.40/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (261.2)

#define DIST_REGULATOR_MAX_ACC_FW (1200)
#define DIST_REGULATOR_MAX_DEC_FW (1200)
#define DIST_REGULATOR_MAX_ACC_BW (1200)
#define DIST_REGULATOR_MAX_DEC_BW (1200)
#define ACC_DEC_DAMPLING (1.6)


float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1};
float speed_controller_right_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6};
float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60};

float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1};
float speed_controller_left_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6};
float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60};

#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.02)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (2.5)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm (25)
#define COMMAND_MANAGER_ALIGN_ONLY_EXIT_ANGLE_THRESHOLD_RAD COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD/10


#define COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm (3)
Goto::GotoConfiguration preciseGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm, COMMAND_MANAGER_ALIGN_ONLY_EXIT_ANGLE_THRESHOLD_RAD};

#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)
Goto::GotoConfiguration waypointGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm, COMMAND_MANAGER_ALIGN_ONLY_EXIT_ANGLE_THRESHOLD_RAD};

#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = {COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (150/DIST_REGULATOR_KP), 85 };


int main(void)
{
    printf("zob\n");
    Regulator reg(10, 10);
    EncoderSimuation encoders;
    MotorControllerSimulation motorController;

    Regulator angleRegulator(ANGLE_REGULATOR_KP, MAX_SPEED_MM_PER_SEC);
    Regulator distanceRegulator(DIST_REGULATOR_KP, FLT_MAX);

    Pll rightPll( PLL_BANDWIDTH);
    Pll leftPll(PLL_BANDWIDTH);

    Odometry odometry(ENCODERS_WHEELS_DISTANCE_MM, 0, 0);


    AdaptativeSpeedController speedControllerRight(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);
    AdaptativeSpeedController speedControllerLeft(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);

    SimpleAccelerationLimiter angleAccelerationlimiter(ANGLE_REGULATOR_MAX_ACC);

    AccelerationDecelerationLimiter distanceAccelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);



    CommandManager commandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   angleRegulator, distanceRegulator,
                                   &distanceAccelerationLimiter,
                                   nullptr);

    AsservMain mainAsserv( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           commandManager, motorController, encoders, odometry,
                           angleRegulator, distanceRegulator,
                           angleAccelerationlimiter, distanceAccelerationLimiter,
                           speedControllerRight, speedControllerLeft,
                           rightPll, leftPll,
                           nullptr);

    NativeStream::init();

    mainAsserv.mainLoop();

}
