#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "CMDList/CMDList.h"
#include "Regulator.h"

class CommandManager
{

public:
    CommandManager();
    ~CommandManager();

    void setAngleRegulator(Regulator *angle_regulator){m_angle_regulator=angle_regulator;};
    void setDistanceRegulator( Regulator *distance_regulator){m_distance_regulator=distance_regulator;};

    bool addStraightLine(float valueInmm);
    bool addTurn(float angleInDeg);
    bool addGoTo(float posXInmm, float posYInmm);
    bool addGoToEnchainement(float posXInmm, float posYInmm);
    bool addGoToAngle(float posXInmm, float posYInmm);


    void perform(float X_mm, float Y_mm, float theta_rad);
    float getDistanceGoal(){ return m_distRegulatorConsign;}
    float getAngleGoal(){ return m_angleRegulatorConsign;}

//    int getLastCommandStatus() { return lastStatus; }
//    void setLastCommandStatus(int s) { lastStatus = s; }

private:
    CMDList liste; //File d'attente des commandes
    CMD currCMD; //commande courante
    CMD nextCMD; //commande suivante



    Regulator *m_angle_regulator;
    Regulator *m_distance_regulator;

    float m_arrivalAngleThreshold;
    float m_arrivalDistanceThreshold;
    float m_arrivalAngleSpeedThreshold;
    float m_arrivalDistSpeedThreshold;
    float m_lastDistanceSpeed; // Last speed output command of the regulators. Considering this as a good estimation of the current speed
    float m_lastAngleSpeed;

    float m_gotoAngleThreshold;

    float m_angleRegulatorConsign;
    float m_distRegulatorConsign;


    int lastStatus;

    bool areRampsFinished();

    float computeDeltaTheta(float deltaX, float deltaY, float theta_rad); // Calcul de l'angle à parcourir
    float computeDeltaDist(float deltaX, float deltaY); // Calcul de la distance à parcourir
    // GoTo là où on veut
    void computeGoTo(float X_mm, float Y_mm, float theta_rad);
    void computeGoToAngle(float deltaX, float deltaY, float theta_rad);
    void computeEnchainement(); // Tentative d'enchainement
};

#endif
