#ifndef GOTOANGLE_H_
#define GOTOANGLE_H_

#include "Command.h"

class GotoAngle : public Command
{
    public:
        explicit GotoAngle(float consignX_mm, float consignY_mm,
                float arrivalAngleThreshold_rad);

        virtual ~GotoAngle() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        virtual AsservMain::mixing_type_t getMixingType() const { return AsservMain::mixing_type_polar; };

    private:
        float m_consignX_mm;
        float m_consignY_mm;

        float m_arrivalAngleThreshold_rad;
};

#endif /* GOTOANGLE_H_ */
