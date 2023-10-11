#include "NativeStream.h"
#include <msgpack11.hpp>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace msgpack11;

#define PORT     9870

NativeStream::NativeStream(float period)
{
    if ((client_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("Socket creation error \n");
        exit(-1);
    }

    serv_addr_.sin_family = AF_INET;
    serv_addr_.sin_port = htons(PORT);

    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr_.sin_addr)  <= 0)
    {
        printf( "Invalid address / Address not supported \n");
        exit(-1);
    }

    period_ = period;
}

void NativeStream::init(float period)
{
    s_instance = new NativeStream(period);
    SampleStream::setInstance(s_instance);

}

void* NativeStream::sendCurrentStream()
{
    my_msgpack["timestamp"] =  my_msgpack["timestamp"].float32_value() + period_;
    MsgPack msgpack(my_msgpack);


    std::string msgpack_bytes = msgpack.dump();


    sendto(client_fd_, (const char *)msgpack_bytes.c_str(), msgpack_bytes.length(), MSG_CONFIRM, (const struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
    return nullptr;
}


