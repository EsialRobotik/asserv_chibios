#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "Odometry.h"
#include "motorController/MotorController.h"
#include <math.h>
#include "USBStream.h"

#include "hal.h"
#include <chprintf.h>
extern BaseSequentialStream *outputStream;

OldSchoolBlockingDetector::OldSchoolBlockingDetector(
        float dt, MotorController const &motorController, Odometry const &odometry,
        float block_angle_speed_threshold, float block_dist_speed_threshold, float blocking_detected_duration_threshold)
: m_motorController(motorController), m_odometry(odometry)
{
    m_dt = dt;
    m_block_angle_speed_threshold = block_angle_speed_threshold;
    m_block_dist_speed_threshold = block_dist_speed_threshold;
    m_blocking_detected_duration = 0;
    m_blocking_detected_duration_threshold = blocking_detected_duration_threshold;
}

OldSchoolBlockingDetector::~OldSchoolBlockingDetector()
{
}


void OldSchoolBlockingDetector::update()
{
    //DEBUG
//    if (m_blocking_detected_duration > m_blocking_detected_duration_threshold)
//    chprintf(outputStream,"BLOCKED\r\n ");

//    if( m_motorController.getMotorLeftSpeed() != 0 && m_motorController.getMotorRightSpeed() != 0
//        && fabs(m_odometry.getDeltaTheta() < m_block_angle_speed_threshold)
//        && fabs(m_odometry.getDeltaDist() < m_block_dist_speed_threshold)    )

    if (( (fabs(m_motorController.getMotorLeftSpeed()) > 10.0) || (fabs(m_motorController.getMotorRightSpeed()) > 10.0))
           && (
           (
                   (m_motorController.getMotorLeftSpeed() * m_motorController.getMotorRightSpeed() <= 0)  &&
                   (fabs(m_odometry.getDeltaTheta()) < m_block_angle_speed_threshold))
           || (
                   (m_motorController.getMotorLeftSpeed() * m_motorController.getMotorRightSpeed() >= 0)  &&
                   (fabs(m_odometry.getDeltaDist()) < m_block_dist_speed_threshold))
              ))
    {

        m_blocking_detected_duration += m_dt;


        //chprintf(outputStream,"%f DD=%f<%f DT=%f<%f\r\n ", m_blocking_detected_duration, m_odometry.getDeltaDist(), m_block_dist_speed_threshold,  m_odometry.getDeltaTheta(), m_block_angle_speed_threshold);
        //chprintf(outputStream,"%f %f\r\n ", m_motorController.getMotorLeftSpeed(), m_motorController.getMotorRightSpeed());

    }
    else
    {
        m_blocking_detected_duration = 0;
    }

    USBStream::instance()->setMovingIntegralError(fabs(m_odometry.getDeltaTheta()));
    USBStream::instance()->setMovingIntegralErrorThreshold(fabs(m_odometry.getDeltaDist()));
}

bool OldSchoolBlockingDetector::isBlocked() const
{
    chprintf(outputStream,"%f %f\r\n ", m_blocking_detected_duration, m_blocking_detected_duration_threshold);
    if (m_blocking_detected_duration > m_blocking_detected_duration_threshold)
        chprintf(outputStream,"isBlocked\r\n ");
    return (m_blocking_detected_duration > m_blocking_detected_duration_threshold);
}
