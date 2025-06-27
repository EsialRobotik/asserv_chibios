#ifndef SRC_COMMAND_H_
#define SRC_COMMAND_H_

#include "AsservMain.h"


class Regulator;

class Command
{
public:

    typedef enum
    {
        consign_acceleration_limited, consign_wheel_speed
    } consign_type_t;


    typedef struct
    {
        consign_type_t type;

        //  consign in polar mode
        float angle_consign;
        float distance_consign;

        // consign in wheel speed mode
        float right_wheel_consign;
        float left_wheel_consign;
    } consign_t;

    virtual ~Command() {}

    virtual consign_type_t computeInitialConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator) = 0;
    virtual void updateConsign(float X_mm, float Y_mm, float theta_rad, consign_t & consign, const Regulator &angle_regulator, const Regulator &distance_regulator) = 0;
    virtual bool isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator, const Command* nextCommand) = 0;

    virtual bool noStop() const = 0;

    virtual AsservMain::mixing_type_t getMixingType() const = 0;

    
    void setIndex(uint32_t index) {m_index = index;};
    uint32_t getIndex() {return m_index;};

    private:
     uint32_t m_index;
};

#endif /* SRC_COMMAND_H_ */
