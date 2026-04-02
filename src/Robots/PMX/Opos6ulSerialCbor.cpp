#include "ch.h"
#include "hal.h"
#include "AsservMain.h"
#include "Opos6ulSerialCbor.h"

void Opos6ulSerialCbor::decode_cmd(CborStreamStateMachine::cmd_t &cmd)
{
    // Commandes PMX spécifiques
    // TODO: ajouter face_reverse, goto_reverse_nostop, etc.
    // switch (cmd.cmd_type)
    // {
    //     case ...:
    //         break;
    //     default:
    //         SerialCbor::decode_cmd(cmd);
    //         break;
    // }

    // Pour l'instant, on délègue tout à SerialCbor
    SerialCbor::decode_cmd(cmd);
}
