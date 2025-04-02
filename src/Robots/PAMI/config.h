#ifndef SRC_ROBOTS_PRINCESS_CONFIG_H_
#define SRC_ROBOTS_PRINCESS_CONFIG_H_

#include "ch.h"
#include "hal.h"
#include "Odometry.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"


#define ASSERV_THREAD_FREQUENCY (400)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_WHEELS_RADIUS_MM (73.2/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (86.03)
#define ENCODERS_TICKS_BY_TURN (546*4)

#define REGULATOR_MAX_SPEED_MM_PER_SEC (600)
#define WHEELS_MAX_SPEED_MM_PER_SEC (650)

#define DIST_REGULATOR_KP (6)
#define DIST_REGULATOR_MAX_ACC (1200)


#define ANGLE_REGULATOR_KP (330)
#define ANGLE_REGULATOR_MAX_ACC (1000)

#define SPEED_CTRL_LEFT_KP_1 (0.5)
#define SPEED_CTRL_LEFT_KP_2 (0.2)
#define SPEED_CTRL_LEFT_KP_3 (0.1)
#define SPEED_CTRL_LEFT_KI_1 (1.3)
#define SPEED_CTRL_LEFT_KI_2 (1.3)
#define SPEED_CTRL_LEFT_KI_3 (2.0)
#define SPEED_CTRL_LEFT_SPEED_THRES_1 (20)
#define SPEED_CTRL_LEFT_SPEED_THRES_2 (50)
#define SPEED_CTRL_LEFT_SPEED_THRES_3 (60)


#define SPEED_CTRL_RIGHT_KP_1 (0.5)
#define SPEED_CTRL_RIGHT_KP_2 (0.2)
#define SPEED_CTRL_RIGHT_KP_3 (0.1)
#define SPEED_CTRL_RIGHT_KI_1 (1.3)
#define SPEED_CTRL_RIGHT_KI_2 (1.3)
#define SPEED_CTRL_RIGHT_KI_3 (2.0)
#define SPEED_CTRL_RIGHT_SPEED_THRES_1 (20)
#define SPEED_CTRL_RIGHT_SPEED_THRES_2 (50)
#define SPEED_CTRL_RIGHT_SPEED_THRES_3 (60)


#define PLL_BANDWIDTH (100)

#define BLOCKING_DETECTOR_ANGLE_SPEED_THRESHOLD (M_PI/8)
#define BLOCKING_DETECTOR_DIST_SPEED_THRESHOLD (20)
#define BLOCKING_DETECTOR_BLOCKING_DURATION_THRESHOLD (0.5)


#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.02)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (2.5)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm (15)
#define COMMAND_MANAGER_ALIGN_ONLY_EXIT_ANGLE_THRESHOLD_RAD COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD/10


#define COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm (3)
#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)
#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)

extern Odometry *odometry;
extern AsservMain *mainAsserv;
extern CommandManager *commandManager;

extern Regulator *angleRegulator;
extern Regulator *distanceRegulator;

extern SimpleAccelerationLimiter *angleAccelerationlimiter;
extern SimpleAccelerationLimiter *distanceAccelerationLimiter;


#endif /* SRC_ROBOTS_PRINCESS_CONFIG_H_ */
