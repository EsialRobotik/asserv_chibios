#ifndef USBSTREAM_SRC_DATASTREAMTYPE_H_
#define USBSTREAM_SRC_DATASTREAMTYPE_H_

#include <stdint.h>
//#include <climits>

#include "USBStreamType.h"



class USBStream {
public:
	static void init();
	static inline USBStream* instance(){
		return s_instance;
	}

	void* SendCurrentStream();


	inline void setValue(float a) {m_currentStruct.tempPhaseA = a;}

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
