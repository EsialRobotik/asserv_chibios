#ifndef USB_STREAM_H_
#define USB_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"


class USBStream: public SampleStream
{
public:
    static void init(float period);

    static USBStream* instance()
    {
        return s_instance;
    }

    virtual void* sendCurrentStream();

private:

    USBStream(float period);
    virtual ~USBStream() {};


    static inline USBStream* s_instance = nullptr;


    uint8_t *m_currentPtr;

    void getEmptyBuffer();
    void sendFullBuffer();

    float period_;

};

#endif /* USB_STREAM_H_ */
