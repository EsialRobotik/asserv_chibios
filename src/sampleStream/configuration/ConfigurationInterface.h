#ifndef CONFIGURATION_ITF_H_
#define CONFIGURATION_ITF_H_


#include "qcbor/qcbor_encode.h"
#include "qcbor/qcbor_decode.h"
#include "qcbor/qcbor_spiffy_decode.h"


class Configuration
{
public:
    virtual ~Configuration(){}

    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx) = 0;


    virtual void applyConfiguration(QCBORDecodeContext &decodeCtx) = 0;
};


#endif /* CONFIGURATION_ITF_H_ */
