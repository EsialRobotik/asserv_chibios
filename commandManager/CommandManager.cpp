#include "CommandManager.h"
#include <math.h>
#include "USBStream.hpp"
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>

extern BaseSequentialStream *outputStream;

//GDI
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

CommandManager::CommandManager():
liste()
{
    currCMD.type = CMD_NULL;
    nextCMD.type = CMD_NULL;
    lastStatus = 2;
    m_angle_regulator = 0;
    m_distance_regulator = 0;
    m_arrivalAngleThreshold = 0.1;
    m_arrivalDistanceThreshold = 1;
    m_arrivalAngleSpeedThreshold = 0.01;
    m_arrivalDistSpeedThreshold = 1;
    m_angleRegulatorConsign = 0;
    m_distRegulatorConsign = 0;
    m_gotoAngleThreshold = M_PI/8;
    m_gotoNextConsignDist = 50;
}

CommandManager::~CommandManager()
{
}

bool CommandManager::addStraightLine(float valueInmm)
{
    lastStatus = 0;
    return liste.enqueue(CMD_GO , valueInmm, 0);
}

bool CommandManager::addTurn(float angleInRad)
{
    lastStatus = 0;
    return liste.enqueue(CMD_TURN , angleInRad , 0);
}

bool CommandManager::addGoTo(float posXInmm, float posYInmm)
{
    lastStatus = 0;
    return liste.enqueue(CMD_GOTO , posXInmm , posYInmm);
}

bool CommandManager::addGoToEnchainement(float posXInmm, float posYInmm)
{
    lastStatus = 0;
    return liste.enqueue(CMD_GOTOENCHAIN , posXInmm, posYInmm);
}

bool CommandManager::addGoToAngle(float posXInmm, float posYInmm)
{
    lastStatus = 0;
    return liste.enqueue(CMD_GOTOANGLE, posXInmm, posYInmm);
}


/*
 * Fonction de mise a jour...
 */
void CommandManager::perform(float X_mm, float Y_mm, float theta_rad)
{
    if (!areRampsFinished(X_mm, Y_mm))
    {

        if (currCMD.type == CMD_GO || currCMD.type == CMD_TURN) {  // On avance ou on tourne sur place
            return; //Dans ce cas, on attend simplement d'etre arrive :)
        } else if (currCMD.type == CMD_GOTO) { // On est en plein GoTo, donc on est en train de se planter et on ajuste
            computeGoTo(X_mm, Y_mm, theta_rad);
        } else if (currCMD.type == CMD_GOTOANGLE) { // On est en plein GoTo en angle, donc on est en train de se planter et on ajuste
            computeGoToAngle(X_mm, Y_mm, theta_rad);
        } else if (currCMD.type == CMD_GOTOENCHAIN) { // Là, on est vraiment en train de se planter parce qu'on veut enchainer les consignes
        	computeEnchainement(X_mm, Y_mm, theta_rad);
        }

        if (nextCMD.type == CMD_NULL) { // On a pas chargé de consigne suivante
            nextCMD = liste.dequeue(); // Du coup, on essaie...
        }

    }
    else
    {

        // ToDo Réfléchir à l'enchainement de commande pour ne pas s'arrêter au moment de calculer la suivante
        if (nextCMD.type != CMD_NULL) { //On a déjà chargée une consigne
            currCMD = nextCMD; // La consigne suivante devient la consigne courante
            nextCMD = liste.dequeue(); // On essaye de récupérer la prochaine consigne
        } else {
            currCMD = liste.dequeue(); // On prend la consigne suivante immédiatement
        }


        if (currCMD.type == CMD_NULL) {  // S'il n'y a plus de commande, on est arrivé à bon port
            return; // Il n'y en a pas...
        }

        if (currCMD.type == CMD_GO) {  // On avance ou on recule de la consigne
            m_distRegulatorConsign += currCMD.value;
        } else if (currCMD.type == CMD_TURN) {   // On tourne de la consigne
        	m_angleRegulatorConsign += currCMD.value;
        } else if (currCMD.type == CMD_GOTO) {   // On appel computeGoTo qui se débrouille pour aller en (x,y)
            computeGoTo(X_mm, Y_mm, theta_rad);
        } else if (currCMD.type == CMD_GOTOENCHAIN) {
            computeEnchainement(X_mm, Y_mm, theta_rad); //On va tenter d'enchainer la consigne suivante
        } else if (currCMD.type == CMD_GOTOANGLE) { // On appel computeGoToAngle qui se débrouille pour s'aligner avec (x,y)
            computeGoToAngle(X_mm, Y_mm, theta_rad);
        }
    }
}

/*
 * On a une commande GoTo(x,y) avec x et y deux points dans le repère du robot
 * (0,0) est la position initiale du robot après calage bordure
 * TODO décider de cette connerie
 */
void CommandManager::computeGoTo(float X_mm, float Y_mm, float theta_rad)
{
	USBStream::instance()->setXGoal(currCMD.value);
	USBStream::instance()->setYGoal(currCMD.secValue);

    float deltaX = currCMD.value - X_mm; // Différence entre la cible et le robot selon X
    float deltaY = currCMD.secValue - Y_mm;  // Différence entre la cible et le robot selon Y

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    float deltaDist = computeDeltaDist(deltaX, deltaY);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    float deltaTheta = computeDeltaTheta(deltaX, deltaY, theta_rad);

    float projectedDist = deltaDist * cos(deltaTheta);


    //TODO JGU: Fix pour ne pas se retourner après avoir dépassé un point sur un overshoot!
    //    mais y'a un bug, si le point est proche mais sur le coté, on va juste avancer !!!!

    if (deltaDist < 50/*Config::returnThreshold*/)
    {
    	m_distRegulatorConsign = projectedDist + m_distance_regulator->getAccumulator();
    }
    else
    {
        m_angleRegulatorConsign = deltaTheta + m_angle_regulator->getAccumulator();

        if (fabs(deltaTheta) < m_gotoAngleThreshold)
        	m_distRegulatorConsign = deltaDist + m_distance_regulator->getAccumulator();
    }
}

/*
 * On a une commande GoToAngle(x,y), on veut que le cap du robot pointe vers ce point
 */
void CommandManager::computeGoToAngle(float X_mm, float Y_mm, float theta_rad)
{

    float deltaX = currCMD.value - X_mm; // Différence entre la cible et le robot selon X
    float deltaY = currCMD.secValue - Y_mm;  // Différence entre la cible et le robot selon Y

    // Angle à parcourir
    float deltaTheta = computeDeltaTheta(deltaX, deltaY, theta_rad);

    m_angleRegulatorConsign = deltaTheta + m_angle_regulator->getAccumulator();
}

/*
 * Calcul de l'angle à parcourir par le robot, ça sert souvent...
 */
float CommandManager::computeDeltaTheta(float deltaX, float deltaY, float theta_rad)
{

    // Cap que doit atteindre le robot
	float thetaCible = atan2(deltaY, deltaX);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
	float deltaTheta = thetaCible - theta_rad;

    // On ajuste l'angle à parcourir pour ne pas faire plus d'un demi-tour
    // Exemple, tourner de 340 degrés est plus chiant que de tourner de -20 degrés
    if (deltaTheta > M_PI) {
        deltaTheta -= 2.0 * M_PI;
    } else if (deltaTheta < -M_PI) {
        deltaTheta += 2.0 * M_PI;
    }

    return deltaTheta;
}

float CommandManager::computeDeltaDist(float deltaX, float deltaY)
{
    // On a besoin de min et max pour le calcul de la racine carrée
    float max = fabs(deltaX) > fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);
    float min = fabs(deltaX) <= fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    if (max != 0) {
        return (max * sqrt(1.0 + (min / max) * (min / max)));
    } else {
        return 0;
    }
}

void CommandManager::computeEnchainement(float X_mm, float Y_mm, float theta_rad)
{
	float X_goto = currCMD.value;
	float Y_goto = currCMD.secValue;

	float X_goal = X_goto;
	float Y_goal = Y_goto;

	/*
	 * En dynamique, la consigne  est l'intersection entre une ligne qui part du centre du robot vers le prochain goto
     *    et un cercle de rayon m_gotoNextConsignDist centré sur le robot.
     *  Sauf si la distance entre le robot et la prochaine consigne est inférieur a m_gotoNextConsignDist
     *  	(ce cas de figure arrive quand il n'y a plus d'autres consignes a enchainer )
	 */
	if(computeDeltaDist(X_goto-X_mm, Y_goto-Y_mm) > m_gotoNextConsignDist)
	{
		float angle = M_PI/2;

		if ((X_goto - X_mm) != 0) // with an angle of M_PI/2 an divide by zero will occur. So handle this special case.
		{
			// Find a linear function that go from the current position to goal ...
			float slope = (Y_goto - Y_mm) / (X_goto - X_mm); // offset of the function is useless

			// ... then find the angle between the previous linear function and a linear function y=0x+X_mm (parallel to the x abscissa)
			// see https://fr.wikipedia.org/wiki/Propri%C3%A9t%C3%A9s_m%C3%A9triques_des_droites_et_des_plans#Angles_de_deux_droites
			angle = atan(abs(slope));
		}

		// Correct the angle if we are in the left side of the trigonometric circle
		if(X_mm > X_goto)
			angle = M_PI - angle;
		// Correct the sign of the angle if we are in the ]-Pi;0[ side of the trigonometric circle
		if(Y_mm > Y_goto)
			angle = -angle;

		// Then find a (x,y) point that will be our current goal
		X_goal = X_mm + cosf(angle)*m_gotoNextConsignDist;
		Y_goal = Y_mm + sinf(angle)*m_gotoNextConsignDist;
	}

//	chprintf(outputStream, "gt(%.2f, %.2f) r(%.2f, %.2f) go(%.2f, %.2f)\r\n",X_goto, Y_goto, X_mm, Y_mm,X_goal, Y_goal);

	USBStream::instance()->setXGoal(X_goal);
	USBStream::instance()->setYGoal(Y_goal);


	// Finally, apply some simple GOTO command
    float deltaX = X_goal - X_mm; // Différence entre la cible et le robot selon X
    float deltaY = Y_goal - Y_mm;  // Différence entre la cible et le robot selon Y

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    float deltaDist = computeDeltaDist(deltaX, deltaY);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    float deltaTheta = computeDeltaTheta(deltaX, deltaY, theta_rad);

	m_angleRegulatorConsign = deltaTheta + m_angle_regulator->getAccumulator();
	m_distRegulatorConsign = deltaDist + m_distance_regulator->getAccumulator();
}

bool CommandManager::isGoalReach()
{
	return 			fabs(m_angle_regulator->getError()) <= m_arrivalAngleThreshold
    			&& 	fabs(m_distance_regulator->getError()) <= m_arrivalDistanceThreshold
//				&&  fabs(m_lastDistanceSpeed) <= m_arrivalDistSpeedThreshold
//    			&& 	fabs(m_lastAngleSpeed) <= m_arrivalAngleSpeedThreshold
				;
}

bool CommandManager::areRampsFinished(float X_mm, float Y_mm)
{

    if (currCMD.type!= CMD_GOTOENCHAIN)
    {
    	return isGoalReach();
    }
    else if(currCMD.type == CMD_GOTOENCHAIN )
    {
        if (nextCMD.type == CMD_GOTO || nextCMD.type == CMD_GOTOENCHAIN)
        {
        	/* Si la prochaine consigne est enchainable, la consigne suivante
        	 *  est l'intersection entre une ligne qui part du centre du robot vers le prochain goto
        	 *    et un cercle de rayon m_gotoNextConsignDist centré sur le robot.
        	 *
        	 * Donc ici, si la distance vers le prochain point est plus petite que le rayon m_gotoNextConsignDist
        	 * 	 on enchaine sur la commande suivante...
        	 */
        	 float deltaX = currCMD.value - X_mm; // Différence entre la cible et le robot selon X
        	 float deltaY = currCMD.secValue - Y_mm;  // Différence entre la cible et le robot selon Y

        	 float deltaDist = computeDeltaDist(deltaX, deltaY);
        	 return (deltaDist < m_gotoNextConsignDist );
        }
        else
        	return isGoalReach(); // Si la consigne suivante n'est pas enchainable, il faut vraiment atteindre la consigne..

    }
    else
    	return false;
}



