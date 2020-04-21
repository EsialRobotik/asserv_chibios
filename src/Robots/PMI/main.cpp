#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#include "util/asservMath.h"
#include "util/chibiOsAllocatorWrapper.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "Encoders/QuadratureEncoder.h"
#include "motorController/Md22.h"
#include "Odometry.h"
#include "USBStream.h"
#include "SlopeFilter.h"
#include "Pll.h"


#define ASSERV_THREAD_FREQUENCY (500)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_WHEELS_RADIUS_MM (47.2/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (297)
#define ENCODERS_TICKS_BY_TURN (1024*4)


#define MAX_SPEED_MM_PER_SEC (500)

#define DIST_REGULATOR_KP (9)
#define DIST_REGULATOR_MAX_DELTA (8/ASSERV_THREAD_PERIOD_S)
#define ANGLE_REGULATOR_KP (1400)
#define ANGLE_REGULATOR_MAX_DELTA (8/ASSERV_THREAD_PERIOD_S)

float speed_controller_right_Kp[NB_PI_SUBSET] = {0.25, 0.25, 0.25};
float speed_controller_right_Ki[NB_PI_SUBSET] = {0.45, 0.45, 0.45};
float speed_controller_right_speed_set[NB_PI_SUBSET] = {500.0, 500.0, 500.0};

float speed_controller_left_Kp[NB_PI_SUBSET] = {0.25, 0.25, 0.25};
float speed_controller_left_Ki[NB_PI_SUBSET] = {0.45, 0.45, 0.45};
float speed_controller_left_speed_set[NB_PI_SUBSET] = {500.0, 500.0, 500.0};

#define PLL_BANDWIDTH (250)


#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.1)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (1)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_CHAIN_NEXT_CMD_DIST_mm (50)




QuadratureEncoder encoders(true, true, true);
Md22::I2cPinInit ESIALCardPinConf_SCL_SDA = {GPIOB, 6, GPIOB, 7};
Md22 md22MotorController(true,true,false, &ESIALCardPinConf_SCL_SDA, 100000);

Regulator angleRegulator(ANGLE_REGULATOR_KP, MAX_SPEED_MM_PER_SEC);
Regulator distanceRegulator(DIST_REGULATOR_KP, MAX_SPEED_MM_PER_SEC);

Odometry odometry(ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

SpeedController speedControllerRight(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_speed_set, 100, MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);
SpeedController speedControllerLeft(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_speed_set, 100, MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);

Pll rightPll(PLL_BANDWIDTH);
Pll leftPll(PLL_BANDWIDTH);

SlopeFilter angleSlopeFilter(ANGLE_REGULATOR_MAX_DELTA);
SlopeFilter distSlopeFilter(DIST_REGULATOR_MAX_DELTA);

CommandManager commandManager(COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm,
		COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_CHAIN_NEXT_CMD_DIST_mm,
		angleRegulator, distanceRegulator);

AsservMain mainAsserv(ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
		ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
		commandManager, md22MotorController, encoders, odometry,
		angleRegulator, distanceRegulator,
		angleSlopeFilter, distSlopeFilter,
		speedControllerRight, speedControllerLeft,
		rightPll, leftPll);


/*
 *  As the dynamic allocation is disabled after init,
 *  use this semaphore to ensure that init is finished before
 *  disabling further dynamic allocation
 */
static binary_semaphore_t asservStarted_semaphore;


static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
    (void) arg;
    chRegSetThreadName("AsservThread");

    md22MotorController.init();
    encoders.init();
    encoders.start();
    USBStream::init();

    chBSemSignal(&asservStarted_semaphore);

    mainAsserv.mainLoop();
}


THD_WORKING_AREA(wa_shell, 1024);
THD_WORKING_AREA(wa_controlPanel, 256);
THD_FUNCTION(ControlPanelThread, p);
THD_FUNCTION(asservCommandSerial, p);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);

void asservCommandSerial();


BaseSequentialStream *outputStream;
int main(void)
{
    halInit();
    chSysInit();

    chBSemObjectInit(&asservStarted_semaphore, true);
    chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);
    chBSemWait(&asservStarted_semaphore);

    sdStart(&SD2, NULL);
    shellInit();

    outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);

    // Custom commands
    const ShellCommand shellCommands[] = { { "asserv", &(asservCommandUSB) }, { nullptr, nullptr } };
    ShellConfig shellCfg =
    {
        /* sc_channel */outputStream,
        /* sc_commands */shellCommands,
#if (SHELL_USE_HISTORY == TRUE)
        /* sc_histbuf */history_buffer,
        /* sc_histsize */sizeof(history_buffer),
#endif
#if (SHELL_USE_COMPLETION == TRUE)
        /* sc_completion */completion_buffer
#endif
    };

#ifdef ENABLE_SHELL
    bool startShell = true;
#else
    bool startShell = false;
#endif
    if (startShell)
    {
        thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
        chRegSetThreadNameX(shellThd, "shell");

        // Le thread controlPanel n'a de sens que quand le shell tourne ?
        thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO, ControlPanelThread, nullptr);
        chRegSetThreadNameX(controlPanelThd, "controlPanel");
    }
    else
    {
        thread_t *asserCmdSerialThread = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, asservCommandSerial, nullptr);
        chRegSetThreadNameX(asserCmdSerialThread, "asserv Command serial");
    }

    deactivateHeapAllocation();

    chThdSetPriority(LOWPRIO);
    while (true)
    {
        palClearPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
        palSetPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
    }
}


void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv)
{
    auto printUsage = []()
    {
        chprintf(outputStream,"Usage :");
        chprintf(outputStream," - asserv enablemotor 0|1\r\n");
        chprintf(outputStream," - asserv enablepolar 0|1\r\n");
        chprintf(outputStream," - asserv coders \r\n");
        chprintf(outputStream," - asserv reset \r\n");
        chprintf(outputStream," - asserv motorspeed [r|l] speed \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time] \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv robotangspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv speedcontrol [r|l] [Kp] [Ki] \r\n");
        chprintf(outputStream," - asserv angleSlope delta_speed \r\n");
        chprintf(outputStream," - asserv distSlope delta_speed \r\n");
        chprintf(outputStream," ------------------- \r\n");
        chprintf(outputStream," - asserv addangle angle_rad \r\n");
        chprintf(outputStream," - asserv anglereset\r\n");
        chprintf(outputStream," - asserv anglecontrol Kp\r\n");
        chprintf(outputStream," ------------------- \r\n");
        chprintf(outputStream," - asserv adddist mm \r\n");
        chprintf(outputStream," - asserv distreset\r\n");
        chprintf(outputStream," - asserv distcontrol Kp\r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv addgoto X Y\r\n");
        chprintf(outputStream," - asserv gototest\r\n");
    };
    (void) chp;

    if (argc == 0)
    {
        printUsage();
        return;
    }

    if (!strcmp(argv[0], "wheelspeedstep"))
    {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);
        int time = atoi(argv[3]);
        chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        chprintf(outputStream, "setting wheel %s to speed %.2f rad/s for %d ms \r\n", (side == 'r') ? "right" : "left", speedGoal, time);

        float speedRight = speedGoal;
        float speedLeft = 0;
        if (side == 'l')
        {
            speedLeft = speedGoal;
            speedRight = 0;
        }

        mainAsserv.setWheelsSpeed(speedRight, speedLeft);
        chThdSleepMilliseconds(time);
        mainAsserv.setWheelsSpeed(0, 0);
    }
    else if (!strcmp(argv[0], "robotfwspeedstep"))
    {
        float speedGoal = atof(argv[1]);
        int time = atoi(argv[2]);
        chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        mainAsserv.setRegulatorsSpeed(speedGoal, 0);
        chThdSleepMilliseconds(time);
        mainAsserv.setRegulatorsSpeed(0, 0);
    }
    else if (!strcmp(argv[0], "robotangspeedstep"))
    {
        float speedGoal = atof(argv[1]);
        int time = atoi(argv[2]);
        chprintf(outputStream, "setting angle robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        mainAsserv.setRegulatorsSpeed(0, speedGoal);
        chThdSleepMilliseconds(time);
        mainAsserv.setRegulatorsSpeed(0, 0);
    }
    else if (!strcmp(argv[0], "speedcontrol"))
    {
        char side = *argv[1];
        float Kp = atof(argv[2]);
        float Ki = atof(argv[3]);

        chprintf(outputStream, "setting speed control Kp:%.2f Ki:%.2f to side %c \r\n", Kp, Ki, side);

        if (side == 'r')
            speedControllerRight.setGains(Kp, Ki);
        else if (side == 'l')
            speedControllerLeft.setGains(Kp, Ki);
    }
    else if (!strcmp(argv[0], "angleSlope"))
    {
        float slope = atof(argv[1]);
        chprintf(outputStream, "setting angle slope delta %.2f \r\n", slope);

        angleSlopeFilter.setSlope(slope);
    }
    else if (!strcmp(argv[0], "distSlope"))
    {
        float slope = atof(argv[1]);
        chprintf(outputStream, "setting distance slope delta %.2f \r\n", slope);

        distSlopeFilter.setSlope(slope);
    }
    else if (!strcmp(argv[0], "addangle"))
    {
        float angle = atof(argv[1]);
        chprintf(outputStream, "Adding angle %.2frad \r\n", angle);

        mainAsserv.resetToNormalMode();
        commandManager.addTurn(angle);
    }
    else if (!strcmp(argv[0], "anglereset"))
    {
        chprintf(outputStream, "Reseting angle accumulator \r\n");
        angleRegulator.reset();
    }
    else if (!strcmp(argv[0], "distreset"))
    {
        chprintf(outputStream, "Reseting distance accumulator \r\n");
        distanceRegulator.reset();
    }
    else if (!strcmp(argv[0], "adddist"))
    {
        float dist = atof(argv[1]);
        chprintf(outputStream, "Adding distance %.2fmm \r\n", dist);

        mainAsserv.resetToNormalMode();
        commandManager.addStraightLine(dist);
    }
    else if (!strcmp(argv[0], "anglecontrol"))
    {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting angle Kp to %.2f \r\n", Kp);

        angleRegulator.setGain(Kp);
    }
    else if (!strcmp(argv[0], "distcontrol"))
    {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting dist Kp to %.2f \r\n", Kp);

        distanceRegulator.setGain(Kp);
    }
    else if (!strcmp(argv[0], "enablemotor"))
    {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));

        mainAsserv.enableMotors(enable);
    }
    else if (!strcmp(argv[0], "coders"))
    {
        int32_t encoderRight, encoderLeft;
        encoders.getEncodersTotalCount(&encoderRight, &encoderLeft);
        chprintf(outputStream, "Encoders count %d %d \r\n", encoderRight, encoderLeft);
    }
    else if (!strcmp(argv[0], "reset"))
    {
        mainAsserv.reset();
    }
    else if (!strcmp(argv[0], "motorspeed"))
    {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);

        chprintf(outputStream, "setting wheel %s to speed %.2f \r\n", (side == 'r') ? "right" : "left", speedGoal);

        if (side == 'l')
            md22MotorController.setMotorLeftSpeed(speedGoal);
        else
            md22MotorController.setMotorRightSpeed(speedGoal);
    }
    else if (!strcmp(argv[0], "enablepolar"))
    {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s polar control\r\n", (enable ? "enabling" : "disabling"));

        mainAsserv.enablePolar(enable);
    }
    else if (!strcmp(argv[0], "addgoto"))
    {
        float X = atof(argv[1]);
        float Y = atof(argv[2]);
        chprintf(outputStream, "Adding goto(%.2f,%.2f) consign\r\n", X, Y);

        mainAsserv.resetToNormalMode();
        commandManager.addGoTo(X, Y);
    }
    else if (!strcmp(argv[0], "gototest"))
    {
//		commandManager.addGoToEnchainement(450,-200);
//		commandManager.addGoToEnchainement(450,-600);
//		commandManager.addGoToEnchainement(300,-400);
//		commandManager.addGoTo(150,0);

        mainAsserv.resetToNormalMode();
        commandManager.addGoToEnchainement(365, -270);
        commandManager.addGoToEnchainement(550, -385);
        commandManager.addGoToEnchainement(490, -590);
        commandManager.addGoToEnchainement(295, -720);
        commandManager.addGoToEnchainement(180, -1000);
        commandManager.addGoToEnchainement(390, -1100);
        commandManager.addGoToEnchainement(550, -900);
        commandManager.addGoToEnchainement(395, -630);
        commandManager.addGoToEnchainement(300, -440);
        commandManager.addGoTo(300, -250);
        commandManager.addGoToAngle(1000, -250);
        commandManager.addStraightLine(-200);

    }
    else
    {
        printUsage();
    }
}


THD_FUNCTION(ControlPanelThread, p)
{
    (void) p;
    void *ptr = nullptr;
    uint32_t size = 0;
    char *firstArg = nullptr;
    char *argv[7];
    while (!chThdShouldTerminateX())
    {
        USBStream::instance()->getFullBuffer(&ptr, &size);
        if (size > 0)
        {
            char *buffer = (char*) ptr;

            /*
             *  On transforme la commande recu dans une version argv/argc
             *    de manière a utiliser les commandes shell déjà définie...
             */
            bool prevWasSpace = false;
            firstArg = buffer;
            int nb_arg = 0;
            for (uint32_t i = 0; i < size; i++)
            {
                if (prevWasSpace && buffer[i] != ' ')
                {
                    argv[nb_arg++] = &buffer[i];
                }

                if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')
                {
                    prevWasSpace = true;
                    buffer[i] = '\0';
                }
                else
                {
                    prevWasSpace = false;
                }
            }

            // On évite de faire appel au shell si le nombre d'arg est mauvais ou si la 1ière commande est mauvaise...
            if (nb_arg > 0 && !strcmp(firstArg, "asserv"))
            {
                asservCommandUSB(nullptr, nb_arg, argv);
            }
            USBStream::instance()->releaseBuffer();
        }
    }
}



static void serialReadLine(char *buffer, unsigned int buffer_size)
{
    unsigned int i;
    for(i=0; i<buffer_size; i++)
    {
        buffer[i] = streamGet(&SD2);
        if ( buffer[i] == '\r' )
            break;
    }
    buffer[i] = '\0';
}



THD_FUNCTION(asservCommandSerial, p)
{
    (void) p;

    /*
     * Commande / Caractères à envoyer sur la série / Paramètres / Effets obtenus

     g%x#%y\n / Goto / x, y : entiers, en mm /Le robot se déplace au point de coordonnée (x, y). Il tourne vers le point, puis avance en ligne droite. L'angle est sans cesse corrigé pour bien viser le point voulu.
     e%x#%y\n / goto Enchaîné / x, y : entiers, en mm / Idem que le Goto, sauf que lorsque le robot est proche du point d'arrivée (x, y), on s'autorise à enchaîner directement la consigne suivante si c'est un Goto ou un Goto enchaîné, sans marquer d'arrêt.
     v%d\n / aVancer / d : entier, en mm / Fait avancer le robot de d mm, tout droit
     t%a\n / Tourner / a : entier, en degrées / Fait tourner le robot de a degrées. Le robot tournera dans le sens trigonométrique : si a est positif, il tourne à gauche, et vice-versa.
     f%x#%y\n / faire Face / x, y : entiers, en mm / Fait tourner le robot pour être en face du point de coordonnées (x, y). En gros, ça réalise la première partie d'un Goto : on se tourne vers le point cible, mais on avance pas.
     h / Halte ! / Arrêt d'urgence ! Le robot est ensuite systématiquement asservi à sa position actuelle. Cela devrait suffire à arrêter le robot correctement. La seule commande acceptée par la suite sera un Reset de l'arrêt d'urgence : toute autre commande sera ignorée.
     r / Reset de l'arrêt d'urgence / Remet le robot dans son fonctionnement normal après un arrêt d'urgence. Les commandes en cours au moment de l'arrêt d'urgence NE sont PAS reprises. Si le robot n'est pas en arrêt d'urgence, cette commande n'a aucun effet.
     --c%s%r / Calage bordure / s : sens du calage bordure, r : robot ('g' : gros ; 'p' : petit) / Effectue un calage bordure. Le robot doit être dans sa zone de départ au début du calage, dirigé vers la case de départ adverse en face de la table. Il doit être assez proche de la bordure derrière lui, et pas trop proche de la bordure sur le côté. A la fin du calage, le robot est prêt à partir pour un match dans sa case de départ.
     Le choix du robot est possible, si on veut que deux robots asservis concourent en même temps sur la même table, pour qu'ils puissent faire un calage bordure en même temps sans se rentrer dedans.

     p / get Position / Récupère la position et le cap du robot sur la connexion i2c, sous la forme de 3 types float (3 * 4 bytes), avec x, y, et a les coordonnées et l'angle du robot.
     S / set Position / applique la nouvelle position du robot

     z / avance de 20 cm
     s / recule de 20 cm
     q / tourne de 45° (gauche)
     d / tourne de -45° (droite)

     M / modifie la valeur d'un paramètre / name, value
     R / réinitialiser l'asserv
     D / dump la config du robot
     G / lire la valeur d'un paramètre / name
     L / recharge la config config.txt
     W / sauvegarde la config courante  config~1.txt = config.default.txt

     I / Active les actions dans la boucle d'asservissement (odo + managers)
     ! / Stoppe actions dans la boucle d'asservissement
     K / desactive le consignController et le commandManager
     J / reactive le consignController et le commandManager

     + / applique une valeur +1 sur les moteurs LEFT
     - / applique une valeur -1 sur les moteurs LEFT
     */

    float consigneValue1 = 0;
    float consigneValue2 = 0;
    char buffer[64];

    chprintf(outputStream, "Started\r\n");


    while(1)
    {
        char readChar = streamGet(&SD2);

        switch (readChar) {

        case 'h': //Arrêt d'urgence
            commandManager.setEmergencyStop();
            chprintf(outputStream, "Arrêt d'urgence ! \r\n");
            break;

        case 'r': //Reset de l'arrêt d'urgence
            commandManager.resetEmergencyStop();
            break;

        case 'z':
            // Go 20cm
            chprintf(outputStream, "consigne avant : 200mm\n");
            commandManager.addStraightLine(200);
            break;

        case 's':
            chprintf(outputStream, "consigne arrière : 200mm\n");
            commandManager.addStraightLine(-200);
            break;

        case 'q':
            chprintf(outputStream, "consigne gauche : 45°\n");
            commandManager.addTurn(degToRad(45));
            break;

        case 'd':
            chprintf(outputStream, "consigne gauche : 45°\n");
             commandManager.addTurn(degToRad(-45));
             break;

        case 'v': //aVance d'un certain nombre de mm
            serialReadLine(buffer, sizeof(buffer));
            sscanf(buffer, "%f", &consigneValue1);
            commandManager.addStraightLine(consigneValue1);
            chprintf(outputStream, "v%f\r\n", consigneValue1);
            break;

        case 't': //Tourne d'un certain angle en degrés
            serialReadLine(buffer, sizeof(buffer));
            sscanf(buffer, "%f", &consigneValue1);
            commandManager.addTurn(degToRad(consigneValue1));
            break;

        case 'f': //faire Face à un point précis, mais ne pas y aller, juste se tourner

            serialReadLine(buffer, sizeof(buffer));
            sscanf(buffer, "%f#%f", &consigneValue1, &consigneValue2);
            commandManager.addGoToAngle(consigneValue1, consigneValue2);
            break;

        case 'g': //Go : va à un point précis
            serialReadLine(buffer, sizeof(buffer));
            sscanf(buffer, "%f#%f", &consigneValue1, &consigneValue2);
            commandManager.addGoTo(consigneValue1, consigneValue2);
            break;

        case 'e': // goto, mais on s'autorise à Enchainer la consigne suivante sans s'arrêter
            serialReadLine(buffer, sizeof(buffer));
            sscanf(buffer, "%f#%f", &consigneValue1, &consigneValue2);
            commandManager.addGoToEnchainement(consigneValue1, consigneValue2);
            break;

        case 'p': //retourne la Position et l'angle courants du robot
            chprintf(outputStream, "x%lfy%lfa%lfs%d\r\n",
                    odometry.getX(), odometry.getY(), odometry.getTheta(),
                    commandManager.getCommandStatus());
            break;
        default:
            chprintf(outputStream, " - unexpected character\r\n");
            break;
        }
    }
}


