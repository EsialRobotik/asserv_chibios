#ifndef SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_
#define SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_

class MotorController
{
    public:
        virtual ~MotorController() {}

        virtual void setMotorRightSpeed(float percentage) = 0;
        virtual void setMotorLeftSpeed(float percentage) = 0;
};

#endif /* SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_ */
