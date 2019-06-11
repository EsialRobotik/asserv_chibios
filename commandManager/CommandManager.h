#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "CMDList/CMDList.h"
#include "Regulator.h"

class CommandManager
{

public:
    CommandManager();
    ~CommandManager();

    void setAngleRegulator(const Regulator *angle_regulator){m_angle_regulator=angle_regulator;};
    void setDistanceRegulator( const Regulator *distance_regulator){m_distance_regulator=distance_regulator;};

    bool addStraightLine(float valueInmm);
    bool addTurn(float angleInDeg);
    bool addGoTo(float posXInmm, float posYInmm);
    bool addGoToEnchainement(float posXInmm, float posYInmm);
    bool addGoToAngle(float posXInmm, float posYInmm);


    void perform(float X_mm, float Y_mm, float theta_rad);
    float getDistanceGoal(){ return m_distRegulatorConsign;}
    float getAngleGoal(){ return m_angleRegulatorConsign;}

private:
    CMDList liste; //File d'attente des commandes
    CMD currCMD; //commande courante
    CMD nextCMD; //commande suivante



    const Regulator *m_angle_regulator;
    const Regulator *m_distance_regulator;

    float m_arrivalAngleThreshold;
    float m_arrivalDistanceThreshold;
    float m_arrivalAngleSpeedThreshold;
    float m_arrivalDistSpeedThreshold;

    float m_gotoAngleThreshold;

    float m_gotoNextConsignDist;

    float m_angleRegulatorConsign;
    float m_distRegulatorConsign;


    int lastStatus;

    bool isGoalReach();
    bool areRampsFinished(float X_mm, float Y_mm);

    float computeDeltaTheta(float deltaX, float deltaY, float theta_rad); // Calcul de l'angle à parcourir
    float computeDeltaDist(float deltaX, float deltaY); // Calcul de la distance à parcourir
    // GoTo là où on veut
    void computeGoTo(float X_mm, float Y_mm, float theta_rad);
    void computeGoToAngle(float deltaX, float deltaY, float theta_rad);
    void computeEnchainement(float X_mm, float Y_mm, float theta_rad); // Tentative d'enchainement
};

#endif
