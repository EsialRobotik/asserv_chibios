#ifndef SRC_BLOCKINGDETECTION_H_
#define SRC_BLOCKINGDETECTION_H_

class BlockingDetection
{
public:
    virtual ~BlockingDetection()
    {
    }

    virtual bool isBlocked() = 0;
};

#endif /* SRC_BLOCKINGDETECTION_H_ */
