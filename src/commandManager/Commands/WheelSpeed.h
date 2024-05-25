#ifndef WHEEL_SPEED_H_
#define WHEEL_SPEED_H_

#include "Command.h"
#include "ch.h"
#include <cstdint>


/* Allow to set a speed to a wheel during a specified time
 *
 * WARNING : this command is for speed PI tuning ONLY and should not been use in other cases
 * */

class WheelSpeed : public Command
{
    public:
        explicit WheelSpeed( float rightWheelSpeed_mmpersec, float leftWheelSpeed_mmpersec, uint32_t stepDuration_ms );
        virtual ~WheelSpeed() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        virtual AsservMain::mixing_type_t getMixingType() const { return AsservMain::mixing_type_direct_speed; }

    private:
        float m_rightWheelSpeed_mmpersec;
        float m_leftWheelSpeed_mmpersec;
        systime_t m_step_endtime;

};

#endif /* WHEEL_SPEED_H_ */
