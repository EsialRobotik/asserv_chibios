#ifndef CONFIGURATION_ITF_H_
#define CONFIGURATION_ITF_H_


#include "cborg/Cbor.h"


class Configuration
{
public:
    virtual ~Configuration(){}

    virtual Cbore & getConfiguration(Cbore & cbor_representation) = 0;
};


#endif /* CONFIGURATION_ITF_H_ */
