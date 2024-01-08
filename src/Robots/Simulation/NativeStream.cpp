#include "NativeStream.h"
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>


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
    timestamp_ = 0.0f;
}

void NativeStream::init(float period)
{
    s_instance = new NativeStream(period);
    SampleStream::setInstance(s_instance);

}

void* NativeStream::sendCurrentStream()
{
    setTimestamp(timestamp_);
    std::string str(description);
    std::stringstream ss (str);
    char delim = ',';

    std::string item;
    int i=0;

   while (getline (ss, item, delim)) {
       my_msgpack[item] = m_currentStruct.array[i];
       i++;
   }

    MsgPack msgpack(my_msgpack);


    std::string msgpack_bytes = msgpack.dump();


    sendto(client_fd_, (const char *)msgpack_bytes.c_str(), msgpack_bytes.length(), MSG_CONFIRM, (const struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
    timestamp_ += period_;
    return nullptr;
}


