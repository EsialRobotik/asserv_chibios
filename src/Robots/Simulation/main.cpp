#include <stdio.h>
#include "Regulator.h"
#include "Pll.h"
#include "sampleStream/SampleStreamInterface.h"
#include "sampleStream/configuration/ConfigurationHandler.h"
#include "robotStub/MotorEncoderSimulator.h"
#include "Odometry.h"
#include "SpeedController/AdaptativeSpeedController.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "commandManager/CommandManager.h"
#include "AsservMain.h"
#include "NativeStream.h"
#include <float.h>
#include <thread>
#include <iostream>
#include <string>
#include <unistd.h>
#include "util/asservMath.h"


#define ASSERV_THREAD_FREQUENCY (600)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_TICKS_BY_TURN (1440*4)

#define ENCODERS_WHEELS_RADIUS_MM (31.40/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (261.2)
#define ENCODERS_TICKS_BY_TURN (1440*4)

#define REGULATOR_MAX_SPEED_MM_PER_SEC (1500)

#define DIST_REGULATOR_KP (5)
#define DIST_REGULATOR_MAX_ACC_FW (1000)
#define DIST_REGULATOR_MAX_DEC_FW (1000)
#define DIST_REGULATOR_MAX_ACC_BW (1000)
#define DIST_REGULATOR_MAX_DEC_BW (1000)
#define ACC_DEC_DAMPLING (1.6)

#define PLL_BANDWIDTH (150)

#define ANGLE_REGULATOR_KP (500)
#define ANGLE_REGULATOR_MAX_ACC (3000)


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


#define COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm (3)
Goto::GotoConfiguration preciseGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm};

#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)
Goto::GotoConfiguration waypointGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm};

#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = {COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (150/DIST_REGULATOR_KP), 85 };


CommandManager *commandManager;
AsservMain *mainAsserv;
MotorEncoderSimulator *motorEncoder;
Odometry *odometry;
void shell();

int main(void)
{
    Regulator angleRegulator(ANGLE_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);
    Regulator distanceRegulator(DIST_REGULATOR_KP, FLT_MAX);

    Pll rightPll( PLL_BANDWIDTH);
    Pll leftPll(PLL_BANDWIDTH);

    odometry = new Odometry(ENCODERS_WHEELS_DISTANCE_MM, 0, 0);


    motorEncoder = new MotorEncoderSimulator(ASSERV_THREAD_PERIOD_S, ENCODERS_WHEELS_RADIUS_MM, ENCODERS_TICKS_BY_TURN, odometry);


    AdaptativeSpeedController speedControllerRight(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);
    AdaptativeSpeedController speedControllerLeft(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);

    SimpleAccelerationLimiter angleAccelerationlimiter(ANGLE_REGULATOR_MAX_ACC);

    AccelerationDecelerationLimiter distanceAccelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, REGULATOR_MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);
//    SimpleAccelerationLimiter distanceAccelerationLimiter(9000000);

    commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   angleRegulator, distanceRegulator,
                                   REGULATOR_MAX_SPEED_MM_PER_SEC, REGULATOR_MAX_SPEED_MM_PER_SEC/3,
                                   &distanceAccelerationLimiter,
//                                   nullptr,
                                   nullptr);

    mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           *commandManager, *motorEncoder, *motorEncoder, *odometry,
                           angleRegulator, distanceRegulator,
                           angleAccelerationlimiter, distanceAccelerationLimiter,
                           speedControllerRight, speedControllerLeft,
                           rightPll, leftPll,
                           nullptr);

    NativeStream::init(ASSERV_THREAD_PERIOD_S);

    thread thread_shell(shell);

    ConfigurationHandler conf_json(&angleRegulator, &distanceRegulator, &angleAccelerationlimiter, &distanceAccelerationLimiter, &speedControllerRight, &speedControllerLeft);
    conf_json.generateRepresentation();

    mainAsserv->mainLoop();
    thread_shell.join();
}


void shell()
{
    printf("shell started \n");
    std::string line;

    while(1)
    {

        std::getline (std::cin,line);

        if( line.c_str()[0] == 'a' )
        {
            printf("Run scenarion \r\n");

//            commandManager->addTurn(degToRad(90));
//            commandManager->addStraightLine(1000);
            commandManager->addGOrbitalTurn(degToRad(90), false, true);


        }
        else if( line.c_str()[0] == 'c' )
        {
            printf("test !\r\n");

            commandManager->addStraightLine(1000);

        }
        else if( line.c_str()[0] == 'b' )
        {
            motorEncoder->backwarkPertubation();

        }
        else if( line.c_str()[0] == 'e' )
        {
            printf("test !\r\n");

            commandManager->addGoTo(5000,0);
            usleep(500000);
            mainAsserv->setEmergencyStop();

        }
        else
        {
            printf("Unknown command %s\n", line.c_str());
        }
    }
}
