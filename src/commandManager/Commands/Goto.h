#ifndef GOTO_H_
#define GOTO_H_

#include "Command.h"

class Goto : public Command
{
    public:

        struct GotoConfiguration
        {
            float gotoReturnThreshold_mm;
            float gotoAngleThreshold_rad;
            float arrivalDistanceThreshold_mm;
        };

        explicit Goto(float consignX_mm, float consignY_mm,
                GotoConfiguration const *configuration,
                float backwardMode = false);

        virtual ~Goto() {};

        virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand);

        virtual bool noStop() const;

        static float computeDeltaDist(float deltaX, float deltaY);
        static float computeDeltaTheta(float deltaX, float deltaY, float theta_rad);

        virtual AsservMain::mixing_type_t getMixingType() const { return AsservMain::mixing_type_polar; };


    private:
        float m_consignX_mm;
        float m_consignY_mm;

        GotoConfiguration const *m_configuration;

        float m_backModeCorrection;

        bool m_alignOnly;
        float m_alignOnlyExitAngleThreshold_rad;
};

#endif /* GOTO_H_ */
