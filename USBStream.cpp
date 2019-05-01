#include "USBStream.hpp"
#include "usbcfg.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "core_cm4.h"

const uint32_t synchroWord = 0xDEADBEEF;

USBStream *USBStream::s_instance = NULL;
USBStream::USBStream()
{
	m_currentPtr = NULL;
	m_timestamp = 0;
	m_bufferSize = 0;
}


void USBStream::init()
{
	s_instance = new USBStream();


	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1000);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);

	s_instance->getEmptyBuffer();
}

void* USBStream::SendCurrentStream()
{
	m_currentStruct.synchro = synchroWord;
	m_currentStruct.timestamp = m_timestamp++;

	sendFullBuffer();

	getEmptyBuffer();

	return m_currentPtr;
}



void USBStream::sendFullBuffer()
{
	if(m_currentPtr != NULL )
	{
		*m_currentPtr = m_currentStruct;
		obqPostFullBuffer(&SDU1.obqueue, sizeof(UsbStreamSample));
	}
}

void USBStream::getEmptyBuffer()
{
	msg_t msg = obqGetEmptyBufferTimeout(&SDU1.obqueue, 0);
	if( msg == MSG_OK)
	{
		m_currentPtr = (UsbStreamSample*) SDU1.obqueue.ptr;
		uint32_t available_size = ((uint32_t)SDU1.obqueue.top - (uint32_t)SDU1.obqueue.ptr);
		chDbgAssert (available_size >= (sizeof(UsbStreamSample)+4), "Not enough space in the free buffer. Did you set a correct USB buffer size ?");
	}
	else
	{
		m_currentPtr = NULL;
	}
}
