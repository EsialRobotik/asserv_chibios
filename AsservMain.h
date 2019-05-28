/*
 * AsservMain.h
 *
 *  Created on: 4 mai 2019
 *      Author: jeff
 */

#ifndef ASSERVMAIN_H_
#define ASSERVMAIN_H_

#include "USBStream.hpp"
#include "Vnh5019.h"
#include "Encoders.h"
#include "SpeedController.h"
#include "Regulator.h"
#include <cstdint>

class AsservMain
{
public:
	explicit AsservMain(float wheelRadius_mm);
	virtual ~AsservMain();


	void init();
	void mainLoop();

	void setMotorsSpeed(float motorLeft, float motorRight);

	void setGainForRightSpeedController(float Kp, float Ki){m_speedControllerRight.setGains(Kp, Ki);};
	void setGainForLeftSpeedController(float Kp, float Ki){m_speedControllerLeft.setGains(Kp, Ki);};

	void setSpeedSlope(float slope){
		m_speedControllerLeft.setSpeedSlope(slope);
		m_speedControllerRight.setSpeedSlope(slope); }

	void setAngleGoal(float goal_rad){ m_angleGoal = goal_rad;}
	void setDistanceGoal(float goal_mm){ m_distanceGoal = goal_mm;}

	void resetAngleAccumulator(){m_angleRegulator.reset();}
	void resetDistAccumulator(){m_distanceRegulator.reset();}


	void setGainForAngleRegulator(float Kp){ m_angleRegulator.setGain(Kp); };

	void enableMotors(bool enable);
	void enablePolar(bool enable);
private:

	float estimateSpeed(int16_t deltaCount);
	float estimateDeltaAngle(int16_t deltaCountRight, int16_t deltaCountLeft );
	float estimateDeltaDistance(int16_t deltaCountRight, int16_t deltaCountLeft );



	Vnh5019 m_motorController;
	Encoders m_encoders;
	SpeedController m_speedControllerRight;
	SpeedController m_speedControllerLeft;
	Regulator m_angleRegulator;
	Regulator m_distanceRegulator;

	float m_encoderWheelsDistance_mm;
	float m_encoderWheelsDistance_ticks;
	float m_encodermmByTicks;
	float m_distanceByEncoderTurn_mm;


	float m_angleGoal;
	float m_distanceGoal;

	uint8_t m_asservCounter;

	bool m_enableMotors;
	bool m_enablePolar;
};

#endif /* ASSERVMAIN_H_ */
