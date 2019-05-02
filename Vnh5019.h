#ifndef VNH5019_H_
#define VNH5019_H_

class Vnh5019
{
public:
	Vnh5019();
	virtual ~Vnh5019();


	void init();
	void setMotor1Speed(float percentage);
	void setMotor2Speed(float percentage);
};

#endif /* VNH5019_H_ */
