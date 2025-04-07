#ifndef ORBITAL_TURN_H_
#define ORBITAL_TURN_H_

#include "Command.h"
#include "Regulator.h"


class OrbitalTurn : public Command
{
    public:
        explicit OrbitalTurn(float angle_rad, bool forward, bool turnToTheRight, float arrivalDistanceThreshold_mm,
                float angle_regulator_normal_max_output, float angle_regulator_orbital_max_output, Regulator &angle_regulator);
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

        float m_angle_regulator_normal_max_output;
        float m_angle_regulator_orbital_max_output;
        Regulator &m_angle_regulator;

};

#endif /* ORBITAL_TURN_H_ */
