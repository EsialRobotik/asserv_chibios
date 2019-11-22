#ifndef SRC_SPEEDCONTROLLER_H_
#define SRC_SPEEDCONTROLLER_H_

class SpeedController {
public:
explicit SpeedController(float speedKp, float speedKi, float outputLimit, float maxInputSpeed, float measureFrequency);
	virtual ~SpeedController(){};

	float update(float actualSpeed);

	void setGains(float Kp, float Ki);

	void setSpeedGoal(float speed);
	float getSpeedGoal(){ return m_speedGoal;};
	float getIntegratedOutput(){return m_integratedOutput;};

	void resetIntegral() {m_integratedOutput = 0;};

private:


	float m_speedGoal;
	float m_integratedOutput;

	float m_speedKp;
	float m_speedKi;

	float m_outputLimit;
	float m_inputLimit;

	float m_measureFrequency;
};

#endif /* SRC_SPEEDCONTROLLER_H_ */
