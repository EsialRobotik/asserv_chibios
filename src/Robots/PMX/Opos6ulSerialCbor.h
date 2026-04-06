#ifndef SRC_ROBOTS_PMX_OPOS6ULSERIALCBOR_H_
#define SRC_ROBOTS_PMX_OPOS6ULSERIALCBOR_H_

#include "Communication/SerialCbor.h"

// SerialCbor spécifique PMX (OPOS6UL).
// Hérite de SerialCbor pour ajouter les commandes propres au PMX
// (face reverse, goto reverse chain, etc.)
class Opos6ulSerialCbor : public SerialCbor
{
public:
    using SerialCbor::SerialCbor;
    void setPositionOutputPeriod(time_conv_t period_ms) { m_positionOutputPeriod_ms = period_ms; }
    void decode_cmd(CborStreamStateMachine::cmd_t &cmd) override;
};

#endif /* SRC_ROBOTS_PMX_OPOS6ULSERIALCBOR_H_ */
