#ifndef TURN_H_
#define TURN_H_

#include "Command.h"

class Turn : public Command
{
    public:
        explicit Turn(float consign_rad, float arrivalDistanceThreshold_mm);
        virtual ~Turn() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        virtual AsservMain::mixing_type_t getMixingType() const { return AsservMain::mixing_type_polar; };

    private:
        float m_angleConsign;
        float m_arrivalAngleThreshold_rad;

};

#endif /* TURN_H_ */
