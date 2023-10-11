#include "USBStream.h"
#include "usbcfg.h"
#include <ch.h>
#include <hal.h>
#include <cstring>
#include "core_cm4.h"
#include <cstring>

USBStream::USBStream(float period)
{
    m_currentPtr = nullptr;
    period_ = period;

}

void USBStream::init(float period)
{
    // USB FS
    palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10)); //USB D+
    palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10)); //USB D-


    s_instance = new USBStream(period);
    SampleStream::setInstance(s_instance);

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    std::memset(SDU1.ob, 0x0F0F0F0F, sizeof(SDU1.ob));
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

void* USBStream::sendCurrentStream()
{
    sendFullBuffer();

    getEmptyBuffer();

    return nullptr;
}


void USBStream::sendFullBuffer()
{
    if (m_currentPtr != nullptr)
    {
        my_msgpack["timestamp"] =  my_msgpack["timestamp"].float32_value() + period_;
        MsgPack msgpack(my_msgpack);

       std::string msgpack_bytes = msgpack.dump();
       memcpy( m_currentPtr, msgpack_bytes.c_str(), msgpack_bytes.length());
       obqPostFullBuffer(&SDU1.obqueue, msgpack_bytes.length());
    }
}

void USBStream::getEmptyBuffer()
{
    msg_t msg = obqGetEmptyBufferTimeout(&SDU1.obqueue, 0);
    if (msg == MSG_OK) {
        m_currentPtr = (uint8_t*) SDU1.obqueue.ptr;
    } else {
        m_currentPtr = NULL;
    }
}



