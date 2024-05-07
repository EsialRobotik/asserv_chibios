#ifndef SRC_ROBOTS_PMX_CONFIG_H_
#define SRC_ROBOTS_PMX_CONFIG_H_

#include "Odometry.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "motorController/Md22.h"



#define ENABLE_SHELL

#define ASSERV_THREAD_FREQUENCY (600) //200=>5ms 300=>3ms 600
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (15) //5 à 200 en 2023

#define ENCODERS_WHEELS_RADIUS_MM (39.93/2.0) // le rayon de vos roues codeuses 39.88, 39.93
#define ENCODERS_WHEELS_DISTANCE_MM (234.4) //distance entre les 2 roues codeuses
#define ENCODERS_TICKS_BY_TURN (16384) //nombre de ticks par tour de vos encodeurs.

#define MAX_SPEED_MM_PER_SEC (1200)
#define SLOW_SPEED_MODE_MAX_SPEED_MM_PER_SEC (100)

#define DIST_REGULATOR_KP (3)//2.7 2.5
#define DIST_REGULATOR_MAX_ACC (2000)
#define DIST_REGULATOR_MIN_ACC (1000)
#define DIST_REGULATOR_HIGH_SPEED_THRESHOLD (200)

#define ANGLE_REGULATOR_KP (600) //480 cheff400 mais trop bas coupe de suisse ?
#define ANGLE_REGULATOR_MAX_ACC (900)


#define SPEED_CTRL_LEFT_KP_1 (0.1)
#define SPEED_CTRL_LEFT_KP_2 (0.1)
#define SPEED_CTRL_LEFT_KP_3 (0.1)
#define SPEED_CTRL_LEFT_KI_1 (1.0)
#define SPEED_CTRL_LEFT_KI_2 (0.8)
#define SPEED_CTRL_LEFT_KI_3 (0.6)
#define SPEED_CTRL_LEFT_SPEED_THRES_1 (20)
#define SPEED_CTRL_LEFT_SPEED_THRES_2 (50)
#define SPEED_CTRL_LEFT_SPEED_THRES_3 (60)


#define SPEED_CTRL_RIGHT_KP_1 (0.1)
#define SPEED_CTRL_RIGHT_KP_2 (0.1)
#define SPEED_CTRL_RIGHT_KP_3 (0.1)
#define SPEED_CTRL_RIGHT_KI_1 (1.0)
#define SPEED_CTRL_RIGHT_KI_2 (0.8)
#define SPEED_CTRL_RIGHT_KI_3 (0.6)
#define SPEED_CTRL_RIGHT_SPEED_THRES_1 (20)
#define SPEED_CTRL_RIGHT_SPEED_THRES_2 (50)
#define SPEED_CTRL_RIGHT_SPEED_THRES_3 (60)


//float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.3, 0.2, 0.1 };
//float speed_controller_right_Ki[NB_PI_SUBSET] = { 3.0, 4.2, 1.5 };
//float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };
//
//float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.3, 0.2, 0.1 }; //0.08
//float speed_controller_left_Ki[NB_PI_SUBSET] = { 3.0, 4.2, 1.5 }; //1.0
//float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };


#define PLL_BANDWIDTH (150) //20 à  cause des codeurs magnetique qui oscillent?  verif pour garder un minimum de variation sur la vitesse

#define BLOCKING_ANGLE_SPEED_THRESHOLD_RAD_PER_S (M_PI/6)//3.6 M_PI/6
#define BLOCKING_DIST_SPEED_THRESHOLD_MM_PER_S (20) //80 20
#define BLOCKING_TIME_THRESHOLD_SEC (0.15) //0.25 0.15
#define MINIMUM_CONSIDERED_SPEED_PERCENT (8) //poucentage minimum en dessous duquel on compte plus la detection

#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.02)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (2.5)

#define COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm (20)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm (3)

#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)

#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)


extern Odometry *odometry;
extern AsservMain *mainAsserv;
extern Md22 *md22MotorController;
extern CommandManager *commandManager;

extern BaseSequentialStream *outputStream;
extern BaseSequentialStream *outputStreamSd4;

extern Regulator *angleRegulator;
extern Regulator *distanceRegulator;




#endif /* SRC_ROBOTS_PMX_CONFIG_H_ */
