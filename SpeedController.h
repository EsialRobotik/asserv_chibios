/*
 * SpeedController.h
 *
 *  Created on: 29 avr. 2019
 *      Author: jeff
 */

#ifndef SRC_SPEEDCONTROLLER_H_
#define SRC_SPEEDCONTROLLER_H_

class SpeedController {
public:
explicit SpeedController(float speedKp, float speedKi, float outputLimit, float maxInputSpeed, float maxDeltaConsign, float measureFrequency);
	virtual ~SpeedController(){};

	float update(float actualSpeed);

	void setGains(float Kp, float Ki);

	void setSpeedGoal(float speed);
	float getSpeedGoal(){ return m_speedGoal;};
	float getLimitedSpeedGoal(){ return m_limitedSpeedGoal;};
	float getIntegratedOutput(){return m_integratedOutput;};

private:
	void resetIntegral() {m_integratedOutput = 0;};


	float m_speedGoal;
	float m_limitedSpeedGoal;
	float m_limitedSpeedGoalPrev;
	float m_integratedOutput;

	float m_speedKp;
	float m_speedKi;

	float m_outputLimit;
	float m_inputLimit;

	float m_deltaConsignMax;
	float m_measureFrequency;
};

#endif /* SRC_SPEEDCONTROLLER_H_ */
