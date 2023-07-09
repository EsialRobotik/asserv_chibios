#ifndef SRC_BLOCKINGDETECTION_H_
#define SRC_BLOCKINGDETECTION_H_

class BlockingDetector
{
public:
    virtual ~BlockingDetector()
    {
    }

    virtual void update() = 0;
    virtual bool isBlocked() const = 0;
    virtual void reset() = 0;
};

#endif /* SRC_BLOCKINGDETECTION_H_ */
