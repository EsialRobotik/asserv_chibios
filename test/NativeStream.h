#ifndef NATIVE_STREAM_H_
#define NATIVE_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"


class NativeStream: public SampleStream
{
public:
    static void init();

    static NativeStream* instance()
    {
        return s_instance;
    }

    virtual void* sendCurrentStream();

private:

    NativeStream();
    virtual ~NativeStream() {};

    static inline NativeStream* s_instance = nullptr;

};

#endif /* NATIVE_STREAM_H_ */
