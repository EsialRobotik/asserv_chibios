#ifndef VNH5019_H_
#define VNH5019_H_

class Vnh5019
{
public:
	explicit Vnh5019(bool invertMotor1, bool invertMotor2);
	virtual ~Vnh5019();


	void init();
	void setMotor1Speed(float percentage);
	void setMotor2Speed(float percentage);

private:
	bool m_invertMotor1;
	bool m_invertMotor2;
};

#endif /* VNH5019_H_ */
