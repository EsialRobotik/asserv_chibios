#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "Odometry.h"
#include "motorController/MotorController.h"
#include <math.h>

#define DEBUG_PRINT 1

#if DEBUG_PRINT == 1
	#include "hal.h"
	#include "USBStream.h"
	#include <chprintf.h>
	extern BaseSequentialStream *outputStream;
	#define debug2(a,b) chprintf(a,b)
	#define debug3(a,b,c) chprintf(a,b,c)
	#define debug4(a,b,c,d) chprintf(a,b,c,d)
#else
	#define debug2(a,b)
	#define debug3(a,b,c)
	#define debug4(a,b,c,d)
#endif

OldSchoolBlockingDetector::OldSchoolBlockingDetector(
        float dt, MotorController const &motorController, Odometry const &odometry,
        float block_angle_speed_threshold, float block_dist_speed_threshold, float blocking_detected_duration_threshold,
        float minimum_considered_speed_percent)
: m_motorController(motorController), m_odometry(odometry)
{
    m_dt = dt;
    m_block_angle_speed_threshold = block_angle_speed_threshold * dt;
    m_block_dist_speed_threshold = block_dist_speed_threshold * dt;
    m_blocking_detected_duration = 0;
    m_blocking_detected_duration_threshold = blocking_detected_duration_threshold;
    m_minimum_considered_speed_percent = minimum_considered_speed_percent;
}

OldSchoolBlockingDetector::~OldSchoolBlockingDetector()
{
}


void OldSchoolBlockingDetector::update()
{
#if DEBUG_PRINT == 1
if (isBlocked())
{
debug2(outputStream,"BLOCKED\r\n ");
}
#endif
    float leftMotorSpeedConsign = m_motorController.getMotorLeftSpeed();
    float rightMotorSpeedConsign = m_motorController.getMotorRightSpeed();
    bool isLeftMotorBackward = std::signbit(leftMotorSpeedConsign);
    bool isRighMotorBackward = std::signbit(rightMotorSpeedConsign);

    bool isRobotGoingStraight = (isLeftMotorBackward == isRighMotorBackward);
    bool isRobotTurning =       (isLeftMotorBackward != isRighMotorBackward);


    /*
     * In order to detect that the robot is stuck the following conditions must be meet :
     *   - one of the motor power must be bigger than a minimal threshold
     *   - if the robot is turning, the angle speed (in rad/sec) is lower than a threshold ( m_block_angle_speed_threshold)
     *   - if the robot is straight, the distance speed (in mm/sec) is lower than a threshold ( m_block_dist_speed_threshold)
     *
     * If theses conditions are met a longer time than m_blocking_detected_duration_threshold, blocking is detected
     */
    if (  (fabs(leftMotorSpeedConsign) > m_minimum_considered_speed_percent || fabs(rightMotorSpeedConsign) > m_minimum_considered_speed_percent)
          &&
           (    (isRobotTurning        && fabs(m_odometry.getDeltaTheta()) < m_block_angle_speed_threshold)
             || (isRobotGoingStraight  && fabs(m_odometry.getDeltaDist()) < m_block_dist_speed_threshold)
           )
       )
    {
        m_blocking_detected_duration += m_dt;
    }
    else
    {
        m_blocking_detected_duration = 0;
    }

    USBStream::instance()->setBlockingDuration(m_blocking_detected_duration);
    USBStream::instance()->setBlockingDetected( (m_blocking_detected_duration > m_blocking_detected_duration_threshold) ? 1.0 : 0.0);
}

bool OldSchoolBlockingDetector::isBlocked() const
{
#if DEBUG_PRINT == 1
	debug4(outputStream,"%f %f\r\n ", m_blocking_detected_duration, m_blocking_detected_duration_threshold);
#endif
    return (m_blocking_detected_duration > m_blocking_detected_duration_threshold);
}
