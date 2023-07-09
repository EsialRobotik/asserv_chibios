#include "NativeStream.h"


NativeStream::NativeStream()
{
}

void NativeStream::init()
{
    s_instance = new NativeStream();
    SampleStream::setInstance(s_instance);

}

void* NativeStream::sendCurrentStream()
{

    return m_currentPtr;
}


