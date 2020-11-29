#ifndef TURN_H_
#define TURN_H_

#include "Command.h"

class Turn : public Command
{
    public:
        explicit Turn(float consign_rad, float arrivalDistanceThreshold_mm);
        Turn( Turn const &command);
        virtual ~Turn() {};

        virtual Turn* cloneIn(Command* ptr) const;

        virtual void computeInitialConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator);

        virtual bool noStop();
    private:
        float m_angleConsign;
        float m_arrivalAngleThreshold_rad;

};

#endif /* TURN_H_ */
