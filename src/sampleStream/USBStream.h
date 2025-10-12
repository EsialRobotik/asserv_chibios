#ifndef USB_STREAM_H_
#define USB_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"
#include "hal.h"
#include "qcbor/qcbor_decode.h"


class ConfigurationHandler;
class CommandHandler;

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

    static void init(UsbStreamPinConf_t *pinConf, uint16_t loopFrequency, ConfigurationHandler *configuration_handler = nullptr, CommandHandler *command_handler = nullptr);

    static USBStream* instance()
    {
        return s_instance;
    }

    virtual void* sendCurrentStream();

    void USBStreamHandleConnection_lowerpriothread();

    void releaseBuffer();
    void getFullBuffer(void** ptr, uint32_t* size);

private:

      explicit USBStream(uint16_t loopFrequency, ConfigurationHandler *configuration_handler, CommandHandler *command_handler);
      virtual ~USBStream() {};

    void sendBuffer(uint8_t const *buffer, uint32_t size, uint32_t synchroWord);


    uint32_t m_timestamp;
    mutex_t m_sample_sending_mutex;
    ConfigurationHandler *m_configuration_handler;
    CommandHandler *m_cmd_handler;
    static inline USBStream* s_instance = nullptr;
    
    uint8_t cbor_buffer[384];
    QCBORDecodeContext m_cborDecoderCtx;


    uint16_t m_loopFrequency;


    void getEmptyBuffer();
    void sendFullBuffer();


};

#endif /* USB_STREAM_H_ */
