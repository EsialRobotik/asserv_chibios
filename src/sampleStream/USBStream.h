#ifndef USB_STREAM_H_
#define USB_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"


class USBStream: public SampleStream
{
public:
    static void init();

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

    USBStream();
    virtual ~USBStream() {};


    uint32_t m_timestamp;
    mutex_t m_sample_sending_mutex;
    static inline USBStream* s_instance = nullptr;




    void getEmptyBuffer();
    void sendFullBuffer();


};

#endif /* USB_STREAM_H_ */
