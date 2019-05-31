#ifndef USBSTREAM_SRC_DATASTREAMTYPE_H_
#define USBSTREAM_SRC_DATASTREAMTYPE_H_

#include <stdint.h>


typedef struct
{
	uint32_t synchro;
	uint32_t timestamp;
	float value1;
	float value2;
	float value3;
	float value4;
	float value5;
	float value6;
	float value7;
	float value8;
	float value9;
	float value10;
	float value11;
	float value12;
	float value13;
	float value14;
	float value15;
	float value16;
	float value17;
	float value18;
	float value19;
	float value20;
	float value21;
	float value22;
	float value23;
}  __attribute__((packed)) UsbStreamSample;


class USBStream {
public:
	static void init();
	static inline USBStream* instance(){
		return s_instance;
	}

	void* SendCurrentStream();


	// Right motor speed control
	inline void setSpeedGoalRight(float speed) {m_currentStruct.value1 = speed;}
	inline void setSpeedEstimatedRight(float speed) {m_currentStruct.value2 = speed;}
	inline void setSpeedOutputRight(float speed) {m_currentStruct.value3 = speed;}
	inline void setSpeedIntegratedOutputRight(float speed) {m_currentStruct.value7 = speed;}
	inline void setLimitedSpeedGoalRight(float speed) {m_currentStruct.value9 = speed;}

	// Left motor speed control
	inline void setSpeedGoalLeft(float speed) {m_currentStruct.value4 = speed;}
	inline void setSpeedEstimatedLeft(float speed) {m_currentStruct.value5 = speed;}
	inline void setSpeedOutputLeft(float speed) {m_currentStruct.value6 = speed;}
	inline void setSpeedIntegratedOutputLeft(float speed) {m_currentStruct.value8 = speed;}
	inline void setLimitedSpeedGoalLeft(float speed) {m_currentStruct.value10 = speed;}

	// Angle regulator
	inline void setAngleGoal(float goal) {m_currentStruct.value11 = goal;}
	inline void setAngleAccumulator(float acc) {m_currentStruct.value12 = acc;}
	inline void setAngleOutput(float output) {m_currentStruct.value13 = output;}

	// Distance regulator
	inline void setDistGoal(float goal) {m_currentStruct.value14 = goal;}
	inline void setDistAccumulator(float acc) {m_currentStruct.value15 = acc;}
	inline void setDistOutput(float output) {m_currentStruct.value16 = output;}

	// Raw Encoder
	inline void setRawEncoderDeltaRight(float delta) {m_currentStruct.value17 = delta;}
	inline void setRawEncoderDeltaLeft(float delta) {m_currentStruct.value18 = delta;}

	// Odometrie
	inline void setOdoX(float x) {m_currentStruct.value19 = x;}
	inline void setOdoY(float y) {m_currentStruct.value20 = y;}
	inline void setOdoTheta(float theta) {m_currentStruct.value21 = theta;}


private:
	USBStream();
	virtual ~USBStream() {};

	void getEmptyBuffer();
	void sendFullBuffer();

	static USBStream* s_instance;

	UsbStreamSample *m_currentPtr;
	UsbStreamSample m_currentStruct;
	uint32_t m_timestamp;
	uint32_t m_bufferSize;
};

#endif /* VITIDRIVE_SRC_DATASTREAM_HPP_ */
