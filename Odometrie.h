#ifndef ODOMETRIE
#define ODOMETRIE

#include <cstdint>

class Odometrie
{

public:
    explicit Odometrie(float encoderWheelsDistance_mm, float initialX = 0, float initialY = 0);
    ~Odometrie(){};

    // Reset de la position du robot
    void resetX(float xval);
    void resetY(float yval);
    void resetTheta();

    void refresh(float m_encoderDeltaRight_mm, float m_encoderDeltaLeft_mm);

    float getX()
    {
        return m_x;   // Renvoie la position en X par rapport au point de départ
    }
    float getY()
    {
        return m_y;   // Renvoie la position en Y par rapport au point de départ
    }
    float getTheta() {
        return m_theta;   // Renvoie l'angle par rapport au cap de départ
    }

private:

    float m_encoderWheelsDistance_mm;
    // Position actuelle
    float m_x, m_y; // En mm
    float m_theta; //En radian
};

#endif
