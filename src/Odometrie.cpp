#include "Odometrie.h"
#include <math.h>
#include "util/constants.h"


Odometrie::Odometrie(float encoderWheelsDistance_mm, float initialX, float initialY )
{
	m_encoderWheelsDistance_mm = encoderWheelsDistance_mm;

    // Initialisation de la position
	m_x = initialX;
	m_y = initialY;
	m_theta = 0;
}

void Odometrie::resetX(float X) {
	m_x = X;
}

void Odometrie::resetY(float Y) {
	m_y = Y;
}

void Odometrie::resetTheta() {
	m_theta = 0;
}       

// Mise à jour de la position du robot
void Odometrie::refresh(float m_encoderDeltaRight_mm, float m_encoderDeltaLeft_mm)
{
    /*
    * deltaDist = la distance parcourue par le robot pendant l'itération = moyenne des distances des codeurs
    * deltaTheta = la variation de l'angle pendant l'itération = rapport de la différence des distances codeurs sur la
    *               distance entre les roues
    */
    float deltaDist = (m_encoderDeltaLeft_mm + m_encoderDeltaRight_mm) / 2;
    float diffCount = m_encoderDeltaRight_mm - m_encoderDeltaLeft_mm;
    double deltaTheta = double(diffCount) /double(m_encoderWheelsDistance_mm); // En radian

    if (diffCount==0) {   // On considère le mouvement comme une ligne droite
        // Mise à jour de la position
    	m_x += deltaDist * cos(m_theta);
    	m_y += deltaDist * sin(m_theta);
    } else { //On approxime en considérant que le robot suit un arc de cercle
        // On calcule le rayon de courbure du cercle
        float R = float(deltaDist) / deltaTheta;
        //Mise à jour de la position
        m_x += R * (-sin(m_theta) + sin(m_theta + deltaTheta));
        m_y += R * (cos(m_theta) - cos(m_theta + deltaTheta));
        // Mise à jour du cap
        m_theta += deltaTheta;

        // On limite le cap à +/- PI afin de ne pouvoir tourner dans les deux sens et pas dans un seul
        if (m_theta > M_PI) {
            m_theta -= 2 * M_PI ;
        } else if (m_theta <= -M_PI) {
            m_theta += 2 * M_PI ;
        }
    }
}
