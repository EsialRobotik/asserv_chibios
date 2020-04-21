#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "CMDList/CMDList.h"
#include "Regulator.h"

enum CommandStatus {
    STATUS_IDLE     = 0,
    STATUS_RUNNING  = 1,
    STATUS_HALTED   = 2,
    STATUS_BLOCKED  = 3,
};

class CommandManager
{
    public:
        explicit CommandManager(float arrivalAngleThreshold_rad, float arrivalDistanceThreshold_mm, float gotoAngleThreshold_rad,
                float gotoNextConsignDist_mm, const Regulator &angle_regulator, const Regulator &distance_regulator);
        ~CommandManager() {};

        /*
         * Commandes ajoutables a la liste des consignes du robot
         */
        bool addStraightLine(float valueInmm);
        bool addTurn(float angleInDeg);
        bool addGoTo(float posXInmm, float posYInmm);
        bool addGoToEnchainement(float posXInmm, float posYInmm);
        bool addGoToAngle(float posXInmm, float posYInmm);

        /*
         * Gestion de l'arret d'urgence
         */
        void setEmergencyStop();
        void resetEmergencyStop();

        /*
         * Mise à jour des consignes de sorties en fonction
         * 	de la nouvelle position du robot
         */
        void update(float X_mm, float Y_mm, float theta_rad);

        /*
         * Sorties du commandManager
         */
        float getDistanceGoal()
        {
            return m_distRegulatorConsign;
        }
        float getAngleGoal()
        {
            return m_angleRegulatorConsign;
        }

        /*
         * Permet au haut niveau de savoir où en est la commande actuelle
         */
        CommandStatus getCommandStatus();

        inline void reset()
        {
            setEmergencyStop();
            resetEmergencyStop();
        }

    private:
        CMDList liste; //File d'attente des commandes
        cmd_t currCMD; //commande courante
        cmd_t nextCMD; //commande suivante

        CommandStatus m_commandStatus;

        const Regulator &m_angle_regulator;
        const Regulator &m_distance_regulator;

        bool m_emergencyStop;

        float m_arrivalAngleThreshold_rad;
        float m_arrivalDistanceThreshold_mm;

        float m_gotoAngleThreshold_rad;

        float m_gotoNextConsignDist_mm;

        float m_angleRegulatorConsign;
        float m_distRegulatorConsign;

        bool isGoalReach();
        bool isGoalReach(float X_mm, float Y_mm);

        bool isBlocked();

        float computeDeltaTheta(float deltaX, float deltaY, float theta_rad); // Calcul de l'angle à parcourir
        float computeDeltaDist(float deltaX, float deltaY); // Calcul de la distance à parcourir
        void computeGoTo(float X_mm, float Y_mm, float theta_rad);
        void computeGoToAngle(float deltaX, float deltaY, float theta_rad);
        void computeEnchainement(float X_mm, float Y_mm, float theta_rad);
};

#endif
