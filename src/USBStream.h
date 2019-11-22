#ifndef USBSTREAM_SRC_DATASTREAMTYPE_H_
#define USBSTREAM_SRC_DATASTREAMTYPE_H_

#include <stdint.h>
#include <cmath>


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


	/*
	 * DIRTY HACK !!
	 *   as uart over usb doesn't seems to like zeros,
	 *   	replace them by NaN that will be replaced by zeros in Plotjuggler
	 */
	inline static void setValue(void *ptr, float value) {
		float *ptrFlt = (float*)ptr;
		if( value == 0.0)
			*ptrFlt = NAN;
		else
			*ptrFlt = value;
	};


	// Right motor speed control
	inline void setSpeedGoalRight(float speed) { setValue( &m_currentStruct.value1, speed); }
	inline void setSpeedEstimatedRight(float speed) { setValue( &m_currentStruct.value2, speed); }
	inline void setSpeedOutputRight(float speed) { setValue( &m_currentStruct.value3, speed); }
	inline void setSpeedIntegratedOutputRight(float speed) { setValue( &m_currentStruct.value7, speed); }

	// Left motor speed control
	inline void setSpeedGoalLeft(float speed) { setValue( &m_currentStruct.value4, speed); }
	inline void setSpeedEstimatedLeft(float speed) { setValue( &m_currentStruct.value5, speed); }
	inline void setSpeedOutputLeft(float speed) { setValue( &m_currentStruct.value6, speed); }
	inline void setSpeedIntegratedOutputLeft(float speed) { setValue( &m_currentStruct.value8, speed); }

	// Angle regulator
	inline void setAngleGoal(float goal) { setValue( &m_currentStruct.value11, goal); }
	inline void setAngleAccumulator(float acc) { setValue( &m_currentStruct.value12, acc); }
	inline void setAngleOutput(float output) { setValue( &m_currentStruct.value13, output); }
	inline void setAngleOutputLimited(float output) { setValue( &m_currentStruct.value9, output); }

	// Distance regulator
	inline void setDistGoal(float goal) { setValue( &m_currentStruct.value14, goal); }
	inline void setDistAccumulator(float acc) { setValue( &m_currentStruct.value15, acc); }
	inline void setDistOutput(float output) { setValue( &m_currentStruct.value16, output); }
	inline void setDistOutputLimited(float output) { setValue( &m_currentStruct.value10, output); }

	// Raw Encoder
	inline void setRawEncoderDeltaRight(float delta) { setValue( &m_currentStruct.value17, delta); }
	inline void setRawEncoderDeltaLeft(float delta) { setValue( &m_currentStruct.value18, delta); }

	// Odometrie
	inline void setOdoX(float x) { setValue( &m_currentStruct.value19, x); }
	inline void setOdoY(float y) { setValue( &m_currentStruct.value20, y); }
	inline void setOdoTheta(float theta) { setValue( &m_currentStruct.value21, theta); }

	// Command Manager
	inline void setXGoal(float x) { setValue( &m_currentStruct.value22, x); }
	inline void setYGoal(float y) { setValue( &m_currentStruct.value23, y); }



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
