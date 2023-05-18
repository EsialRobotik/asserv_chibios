#ifndef SRC_BLOCKINGDETECTOR_OLDSCHOOLDETECTOR_H_
#define SRC_BLOCKINGDETECTOR_OLDSCHOOLDETECTOR_H_

#include <cstdint>

#include "blockingDetector/BlockingDetector.h"

class MotorController;
class Odometry;

class OldSchoolBlockingDetector : public BlockingDetector
{
    public:
        explicit OldSchoolBlockingDetector(
                float dt, MotorController const &motorController, Odometry const &odometry,
                float block_angle_speed_threshold, float block_dist_speed_threshold, float blocking_detected_duration_threshold,
                float minimum_considered_speed_percent = 10);
        virtual ~OldSchoolBlockingDetector();

        virtual bool isBlocked() const;
        virtual void update();

        virtual void reset();

    private:
        float m_dt;
        MotorController const &m_motorController;
        Odometry const &m_odometry;

        float m_blocking_detected_duration;

        float m_block_angle_speed_threshold;
        float m_block_dist_speed_threshold;

        float m_blocking_detected_duration_threshold;

        float m_minimum_considered_speed_percent;
};

#endif /* SRC_BLOCKINGDETECTOR_OLDSCHOOLDETECTOR_H_ */
