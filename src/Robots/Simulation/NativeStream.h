#ifndef NATIVE_STREAM_H_
#define NATIVE_STREAM_H_

#include "SampleStreamInterface.h"
#include "ch.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>



class NativeStream: public SampleStream
{
public:
    static void init(float period);

    static NativeStream* instance()
    {
        return s_instance;
    }

    virtual void* sendCurrentStream();

private:

    NativeStream(float period);
    virtual ~NativeStream() {};

    static inline NativeStream* s_instance = nullptr;

    struct sockaddr_in serv_addr_;
    int client_fd_;
    float period_;
};

#endif /* NATIVE_STREAM_H_ */
