#ifndef ASSERVMAIN_H_
#define ASSERVMAIN_H_

#include "motorController/MotorController.h"
#include "SpeedController.h"
#include "Regulator.h"
#include <cstdint>

class CommandManager;
class Encoders;
class Odometry;
class SlopeFilter;
class Pll;

class AsservMain
{
public:
	explicit AsservMain(uint16_t loopFrequency, uint16_t speedPositionLoopDivisor,
			float wheelRadius_mm, float encoderWheelsDistance_mm, float encodersTicksByTurn,
			CommandManager &commandManager, MotorController &motorController, Encoders &encoders, Odometry &odometrie,
			Regulator &angleRegulator, Regulator &distanceRegulator,
			SlopeFilter &angleRegulatorSlopeFilter, SlopeFilter &distanceRegulatorSlopeFilter,
			SpeedController &speedControllerRight, SpeedController &speedControllerLeft,
			Pll &rightPll, Pll &leftPll);

	virtual ~AsservMain(){};

	void mainLoop();

	void setRegulatorsSpeed(float distSpeed, float angleSpeed);

	void enableMotors(bool enable);
	void enablePolar(bool enable);
private:

	float convertSpeedTommSec(float speed_ticksPerSec);
	float estimateDeltaAngle(int16_t deltaCountRight, int16_t deltaCountLeft );
	float estimateDeltaDistance(int16_t deltaCountRight, int16_t deltaCountLeft );



	MotorController &m_motorController;
	Encoders &m_encoders;
	Odometry &m_odometry;
	SpeedController &m_speedControllerRight;
	SpeedController &m_speedControllerLeft;
	Regulator &m_angleRegulator;
	Regulator &m_distanceRegulator;
	SlopeFilter &m_angleRegulatorSlopeFilter;
	SlopeFilter &m_distanceRegulatorSlopeFilter;
	CommandManager &m_commandManager;
	Pll &m_pllRight;
	Pll &m_pllLeft;


	const float m_distanceByEncoderTurn_mm;
	const float m_encodersTicksByTurn;
	const float m_encodermmByTicks;
	const float m_encoderWheelsDistance_ticks;

	const uint16_t m_loopFrequency;
	const float m_loopPeriod;
	const uint16_t m_speedPositionLoopDivisor;
	uint8_t m_asservCounter;

	float m_distRegulatorOutputSpeedConsign;
	float m_distSpeedLimited;

	float m_angleRegulatorOutputSpeedConsign;
	float m_angleSpeedLimited;

	bool m_enableMotors;
	bool m_enablePolar;
};

#endif /* ASSERVMAIN_H_ */
