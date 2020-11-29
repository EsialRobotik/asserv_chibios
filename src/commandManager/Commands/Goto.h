#ifndef Goto_H_
#define Goto_H_

#include "Command.h"



class Goto : public Command
{
    public:

        struct GotoConfiguration
        {
            float gotoReturnThreshold_mm;
            float gotoAngleThreshold_rad;
            float arrivalDistanceThreshold_mm;
            float arrivalAngleThreshold_rad;
        };

        explicit Goto(float consignX_mm, float consignY_mm,
                GotoConfiguration const *configuration,
                float backwardMode = false);

        Goto( Goto const &command);
        virtual ~Goto() {};

        virtual Goto* cloneIn(Command* ptr) const;

        virtual void computeInitialConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator);
        virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator);

        virtual bool noStop();

        static float computeDeltaDist(float deltaX, float deltaY);
        static float computeDeltaTheta(float deltaX, float deltaY, float theta_rad);
    private:
        float m_consignX_mm;
        float m_consignY_mm;

        GotoConfiguration const *m_configuration;

        float m_backModeCorrection;
};

#endif /* Goto_H_ */
