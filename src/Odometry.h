#ifndef ODOMETRIE
#define ODOMETRIE

#include <cstdint>

class Odometry
{

public:
    explicit Odometry(float encoderWheelsDistance_mm, float initialX = 0, float initialY = 0);
    ~Odometry()
    {
    }
    ;

    // Reset de la position du robot
    void resetX(float xval);
    void resetY(float yval);
    void resetTheta();

    void reset();

    void refresh(float m_encoderDeltaRight_mm, float m_encoderDeltaLeft_mm);

    float getX()
    {
        return m_X_mm;   // Renvoie la position en X par rapport au point de départ
    }
    float getY()
    {
        return m_Y_mm;   // Renvoie la position en Y par rapport au point de départ
    }
    float getTheta()
    {
        return m_theta_rad;   // Renvoie l'angle par rapport au cap de départ
    }

private:

    float m_encoderWheelsDistance_mm;

    // Position actuelle
    float m_X_mm, m_Y_mm; // En mm
    float m_theta_rad; //En radian
};

#endif
