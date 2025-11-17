
#ifndef SRC_COMMUNICATION_CBORSTREAM_CBORSTREAMSTATEMACHINE_H_
#define SRC_COMMUNICATION_CBORSTREAM_CBORSTREAMSTATEMACHINE_H_

#include <cstdint>
#include "CircularBuffer.h"
#include "qcbor/qcbor_decode.h"


class Crc32Calculator;


class CborStreamStateMachine
{
public:

    typedef enum {
    //No param messages
        emergency_stop=10,
        emergency_stop_reset=11,
        normal_speed_acc_mode=15,
        slow_speed_acc_mode=16,
    //Two param messages   
        max_motor_speed=17, // One dummy ID is added in this message to make the decoding part easier
        turn=20,
        straight=21,
    //Three param messages
        face=22,
        goto_front=23,
        goto_back=24,
        goto_nostop=25,
    //Four param messages
        orbital_turn=30
    } cmd_type_t;


    typedef struct {
        cmd_type_t cmd_type;
        int cmd_id;
        float arg1; 
        float arg2; 
        float arg3; 
    } cmd_t;


    explicit CborStreamStateMachine(Crc32Calculator *crc32Calculator);
    virtual ~CborStreamStateMachine() {};

    void push_byte(uint8_t byte);

    bool get_cmd(cmd_t &cmd);


    private:

    typedef enum {
        no_transition,
        to_synchroLookup,
        to_decode,
    } state_transition_t;


    class CborStreamStateInterface
    {
        public:
            virtual ~CborStreamStateInterface() {};

            virtual state_transition_t push_byte(uint8_t byte) = 0;
    };

    class CborStreamState_synchroLookup : public CborStreamStateInterface
    {
        public:
            explicit CborStreamState_synchroLookup();
            virtual ~CborStreamState_synchroLookup() {};

            virtual state_transition_t push_byte(uint8_t byte);

        private:
            uint32_t m_nbBytesOfSynchroWordFound;

            void reset();
    };

    class CborStreamState_decode : public CborStreamStateInterface
    {
        public:
            explicit CborStreamState_decode(Crc32Calculator *crc32Calculator);
            virtual ~CborStreamState_decode() {};

            virtual state_transition_t push_byte(uint8_t byte);

            bool get_cmd(cmd_t &cmd);

        private:
            QCBORDecodeContext m_cborDecoderCtx;
            Crc32Calculator *m_crc32Calculator;
            uint32_t m_size;
            uint32_t m_nbByteOfSizeRead;
            uint32_t m_crc;
            uint32_t m_nbByteOfCrcRead;
            uint8_t  m_payload[64];
            uint32_t m_nbByteOfPayloadRead;
            ring_buffer<cmd_t> cmd_list;
            void reset();
            void validate_payload();
    };

    
    CborStreamState_synchroLookup m_synchroLookupState;
    CborStreamState_decode m_decodeState;
    CborStreamStateInterface *m_currentState;
};


#endif /* SRC_COMMUNICATION_CBORSTREAM_CBORSTREAMSTATEMACHINE_H_ */
