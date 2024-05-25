#ifndef STRAITLINE_H_
#define STRAITLINE_H_

#include "Command.h"

class StraitLine : public Command
{
    public:
        explicit StraitLine(float consign, float arrivalDistanceThreshold_mm);
        virtual ~StraitLine() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        virtual AsservMain::mixing_type_t getMixingType() const { return AsservMain::mixing_type_polar; };

    private:
        float m_straitLineConsign;
        float m_arrivalDistanceThreshold_mm;

};

#endif /* STRAITLINE_H_ */
