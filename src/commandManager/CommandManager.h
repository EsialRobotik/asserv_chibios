#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "CMDList/CMDList.h"
#include "Regulator.h"

class CommandManager
{

public:
   explicit CommandManager(const Regulator &angle_regulator, const Regulator &distance_regulator);
    ~CommandManager();

    /*
     * Commandes ajoutables a la liste des consignes du robot
     */
    bool addStraightLine(float valueInmm);
    bool addTurn(float angleInDeg);
    bool addGoTo(float posXInmm, float posYInmm);
    bool addGoToEnchainement(float posXInmm, float posYInmm);
    bool addGoToAngle(float posXInmm, float posYInmm);

    /*
     * Mise à jour des consignes de sorties en fonction
     * 	de la nouvelle position du robot
     */
    void update(float X_mm, float Y_mm, float theta_rad);

    /*
     * Sorties du commandManager
     */
    float getDistanceGoal(){ return m_distRegulatorConsign;}
    float getAngleGoal(){ return m_angleRegulatorConsign;}

private:
    CMDList liste; //File d'attente des commandes
    CMD currCMD; //commande courante
    CMD nextCMD; //commande suivante



    const Regulator &m_angle_regulator;
    const Regulator &m_distance_regulator;

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
    void computeGoTo(float X_mm, float Y_mm, float theta_rad);
    void computeGoToAngle(float deltaX, float deltaY, float theta_rad);
    void computeEnchainement(float X_mm, float Y_mm, float theta_rad);
};

#endif
