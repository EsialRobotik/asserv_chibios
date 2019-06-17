#ifndef SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_
#define SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_

class MotorController
{
    public:
        virtual ~MotorController() {}

        virtual void setMotor1Speed(float percentage) = 0;
        virtual void setMotor2Speed(float percentage) = 0;
};

#endif /* SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_ */
