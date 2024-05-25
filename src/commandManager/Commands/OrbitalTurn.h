#ifndef ORBITAL_TURN_H_
#define ORBITAL_TURN_H_

#include "Command.h"

class OrbitalTurn : public Command
{
    public:
        explicit OrbitalTurn(float angle_rad, bool forward, bool turnToTheRight, float arrivalDistanceThreshold_mm);
        virtual ~OrbitalTurn() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        virtual AsservMain::mixing_type_t getMixingType() const;
    private:
        float m_angleConsign;
        float m_arrivalAngleThreshold_rad;
        AsservMain::mixing_type_t m_mixing_type;

};

#endif /* ORBITAL_TURN_H_ */
