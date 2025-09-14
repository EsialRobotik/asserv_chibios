#ifndef CONFIGURATION_ITF_H_
#define CONFIGURATION_ITF_H_


#include "qcbor/qcbor_encode.h"


class Configuration
{
public:
    virtual ~Configuration(){}

    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx) = 0;
};


#endif /* CONFIGURATION_ITF_H_ */
