#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "CommandList.h"
#include "Commands/StraitLine.h"
#include "Commands/Goto.h"
#include "Commands/GotoNoStop.h"
#include "Regulator.h"
#include "AsservMain.h"
#include <cstdint>


class Command;
class AccelerationDecelerationLimiter;
class BlockingDetector;


class CommandManager
{
    public:

        typedef enum {
            STATUS_IDLE     = 0,
            STATUS_RUNNING  = 1,
            STATUS_HALTED   = 2,
            STATUS_BLOCKED  = 3,
        } CommandStatus;

        explicit CommandManager(float straitLineArrivalWindows_mm, float turnArrivalWindows_rad,
                Goto::GotoConfiguration &preciseGotoConfiguration, Goto::GotoConfiguration &waypointGotoConfiguration, GotoNoStop::GotoNoStopConfiguration &gotoNoStopConfiguration,
                Regulator &angle_regulator, Regulator &distance_regulator,
                float angle_regulator_normal_max_output, float angle_regulator_orbital_max_output,
                AccelerationDecelerationLimiter *accelerationDecelerationLimiter = nullptr,
                BlockingDetector *blockingDetector = nullptr);
        ~CommandManager() {};

        /*
         * Commandes ajoutables a la liste des consignes du robot
         */
        bool addStraightLine(float valueInmm, uint32_t index = 0);
        bool addTurn(float angleInDeg, uint32_t index = 0);
        bool addGoTo(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGoToWaypoint(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGoToBack(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGoToNoStop(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGoToNoStopBack(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGoToAngle(float posXInmm, float posYInmm, uint32_t index = 0);
        bool addGOrbitalTurn(float angleInDeg, bool forward, bool turnToTheRight, uint32_t index = 0);
        bool addWheelsSpeed(float rightWheelSpeedInmmpersec, float leftWheelSpeedInmmpersec, uint32_t stepDurationInms, uint32_t index = 0);

        /*
         * Gestion de l'arret d'urgence
         */
        void setEmergencyStop();
        void resetEmergencyStop();

        /*
         * Mise à jour des consignes de sorties en fonction
         *     de la nouvelle position du robot
         */
        void update(float X_mm, float Y_mm, float theta_rad);

        Command::consign_t getConsign()
       {
           return m_consign;
       }

        /*
         * Permet au haut niveau de savoir où en est la commande actuelle
         */
        CommandManager::CommandStatus getCommandStatus();
        uint8_t getPendingCommandCount();
        uint32_t getCurrentCommandIndex();

        AsservMain::mixing_type_t getCurrentCommandMixingType() const;

        inline void reset()
        {
            setEmergencyStop();
            resetEmergencyStop();
        }

    private:

        void switchToNextCommand();

        CommandList m_cmdList;
        Command *m_currentCmd;

        float m_straitLineArrivalWindows_mm;
        float m_turnArrivalWindows_rad;
        Goto::GotoConfiguration m_preciseGotoConfiguration;
        Goto::GotoConfiguration m_waypointGotoConfiguration;
        GotoNoStop::GotoNoStopConfiguration m_gotoNoStopConfiguration;
        AccelerationDecelerationLimiter *m_accelerationDecelerationLimiter;

        Regulator &m_angle_regulator;
        Regulator &m_distance_regulator;

        bool m_emergencyStop;
        bool m_blockingDetected;

        float m_angle_regulator_normal_max_output;
        float m_angle_regulator_orbital_max_output;

        BlockingDetector *m_blockingDetector;

        Command::consign_t m_consign;
};

#endif
