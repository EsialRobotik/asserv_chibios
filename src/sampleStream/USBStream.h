#ifndef USB_STREAM_H_
#define USB_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"
#include "hal.h"

class USBStream: public SampleStream
{
public:

    struct UsbStreamPinConf_t
    {
        stm32_gpio_t* dataPlusPin_GPIObase;
        uint8_t dataPlusPin_number;
        uint8_t dataPlusPin_alternate;
        stm32_gpio_t* dataMinusPin_GPIObase;
        uint8_t dataMinusPin_number;
        uint8_t dataMinusPin_alternate;
    };

    static void init(UsbStreamPinConf_t *pinConf, uint16_t loopFrequency);

    static USBStream* instance()
    {
        return s_instance;
    }

    virtual void* sendCurrentStream();

    void sendConfig(uint8_t *configBuffer, uint8_t size);


    typedef void (*usbStreamCallback)(char *buffer, uint32_t size);
    void USBStreamHandleConnection_lowerpriothread(usbStreamCallback callback);

    void releaseBuffer();
    void getFullBuffer(void** ptr, uint32_t* size);

private:

    USBStream(uint16_t loopFrequency);
    virtual ~USBStream() {};


    uint32_t m_timestamp;
    mutex_t m_sample_sending_mutex;
    static inline USBStream* s_instance = nullptr;


    uint16_t m_loopFrequency;


    void getEmptyBuffer();
    void sendFullBuffer();


};

#endif /* USB_STREAM_H_ */
