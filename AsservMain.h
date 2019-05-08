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
#include <cstdint>

class AsservMain
{
public:
	AsservMain();
	virtual ~AsservMain();


	void init();
	void mainLoop();

	void setMotorsSpeed(float motorLeft, float motorRight);

	void setGainForRightSpeedController(float Kp, float Ki){m_speedControllerRight.setGains(Kp, Ki);};
	void setGainForLeftSpeedController(float Kp, float Ki){m_speedControllerLeft.setGains(Kp, Ki);};

private:

	float estimateSpeed(int16_t deltaCount);


	Vnh5019 m_motorController;
	Encoders m_encoders;
	SpeedController m_speedControllerRight;
	SpeedController m_speedControllerLeft;
};

#endif /* ASSERVMAIN_H_ */
