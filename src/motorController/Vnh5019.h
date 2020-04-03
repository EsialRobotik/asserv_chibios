#ifndef VNH5019_H_
#define VNH5019_H_

#include "MotorController.h"

class Vnh5019 : public MotorController
{
public:
	explicit Vnh5019(bool invertMotor1, bool invertMotor2);
	virtual ~Vnh5019() {};


	void init();
	void setMotorRightSpeed(float percentage);
	void setMotorLeftSpeed(float percentage);

private:
	bool m_invertMotor1;
	bool m_invertMotor2;
};

#endif /* VNH5019_H_ */
