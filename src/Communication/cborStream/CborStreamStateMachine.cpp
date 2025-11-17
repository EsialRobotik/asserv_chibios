#include "ch.h"
#include "hal.h"
#include "Communication/Crc/CrcCalculator.h"
#include <chprintf.h>
#include "CborStreamStateMachine.h"
#include "qcbor/qcbor_spiffy_decode.h"

CborStreamStateMachine::CborStreamStateMachine(Crc32Calculator *crc32Calculator) : m_decodeState(crc32Calculator)
{   
    m_currentState = &m_synchroLookupState;
}

void CborStreamStateMachine::push_byte(uint8_t byte)
{
    state_transition_t transition = m_currentState->push_byte(byte);
   
    if( transition == to_synchroLookup)
    {
        m_currentState = &m_synchroLookupState;
    }
    else if( transition == to_decode)
    {
        m_currentState = &m_decodeState;
    }
}

bool CborStreamStateMachine::get_cmd(cmd_t &cmd)
{
    return m_decodeState.get_cmd((cmd));
}

/*
 * Implementation of synchro lookup state : 
 */

CborStreamStateMachine::CborStreamState_synchroLookup::CborStreamState_synchroLookup() 
    : CborStreamStateInterface() 
{
    reset();
};


void CborStreamStateMachine::CborStreamState_synchroLookup::reset()
{
    m_nbBytesOfSynchroWordFound = 0;
}

CborStreamStateMachine::state_transition_t CborStreamStateMachine::CborStreamState_synchroLookup::push_byte(uint8_t byte)
{
    constexpr uint32_t synchroWord = 0xDEADBEEF;

    if( byte == ((uint8_t*)&synchroWord)[m_nbBytesOfSynchroWordFound] ) // Try to synchronize of synchroWord 
    {
        m_nbBytesOfSynchroWordFound++;
    }
    else 
    {
        m_nbBytesOfSynchroWordFound = 0;
    }

    if ( m_nbBytesOfSynchroWordFound == sizeof(synchroWord)) // synchro found.
    {
        reset();
        return to_decode;
    }

    return no_transition;
}


/*
 * Implementation of decode state : 
 */

 CborStreamStateMachine::CborStreamState_decode::CborStreamState_decode(Crc32Calculator *crc32Calculator) 
    : CborStreamStateInterface(), cmd_list(5)
{
    m_crc32Calculator = crc32Calculator;
    reset();
}

 void CborStreamStateMachine::CborStreamState_decode::reset()
{
    m_size = 0;
    m_nbByteOfSizeRead = 0;
    m_crc = 0;
    m_nbByteOfCrcRead = 0;
    m_nbByteOfPayloadRead = 0;
}

CborStreamStateMachine::state_transition_t CborStreamStateMachine::CborStreamState_decode::push_byte(uint8_t byte)
{
    if( m_nbByteOfCrcRead < 4) // First read the CRC 
    {
        ((uint8_t*)&m_crc)[m_nbByteOfCrcRead++] = byte;
    }
    else if( m_nbByteOfSizeRead < 4) // Then the size of the payload
    {
        ((uint8_t*)&m_size)[m_nbByteOfSizeRead++] = byte;
    }
    else if( m_nbByteOfPayloadRead < m_size) // Finaly, get the payload
    {  
        /* Pay attention to not overflow the payload pre-allocated buffer.
         * If read is bigger than the local payload buffer size, just go to synchro_lookup state to avoid useless CRC check.
        */
        if( m_size > sizeof(m_payload) ) 
        {
            return to_synchroLookup;
        }

        m_payload[m_nbByteOfPayloadRead++] = byte;


        if( m_nbByteOfPayloadRead == m_size ) // Here, the payload is completly read
        {
            validate_payload();
            reset();
            return to_synchroLookup;
        }
    }
    return no_transition;
}

void CborStreamStateMachine::CborStreamState_decode::validate_payload()
{
    // First compute CRC
    // crcAcquireUnit(m_crcDriver);
    // crcReset(m_crcDriver);
    // uint32_t computed_crc = crcCalc(m_crcDriver, m_nbByteOfPayloadRead, m_payload);
    // crcReleaseUnit(m_crcDriver);
    uint32_t computed_crc = m_crc32Calculator->compute(m_payload, m_nbByteOfPayloadRead);


    if( computed_crc == m_crc)
    {
        // Then decode using QCbor Decoder
        UsefulBufC encoded_cbor = {.ptr = m_payload, .len = m_size};
        QCBORDecode_Init(&m_cborDecoderCtx, encoded_cbor, QCBOR_DECODE_MODE_NORMAL);
        QCBORDecode_EnterMap(&m_cborDecoderCtx, NULL);

        // Parse the decoded the cbor message depending on its contents
        cmd_t cmd;
        int64_t BigInteger;
        QCBORDecode_GetInt64(&m_cborDecoderCtx, &BigInteger);
        cmd.cmd_type = (cmd_type_t)BigInteger;

        if( cmd.cmd_type >= max_motor_speed)
        {
            double bigFloat;
            QCBORDecode_GetDouble(&m_cborDecoderCtx, &bigFloat);
            cmd.arg1 = float(bigFloat);

            if( cmd.cmd_type >= face)
            {
                QCBORDecode_GetDouble(&m_cborDecoderCtx, &bigFloat);
                cmd.arg2 = float(bigFloat);
            }


            if( cmd.cmd_type >= orbital_turn)
            {
                QCBORDecode_GetDouble(&m_cborDecoderCtx, &bigFloat);
                cmd.arg3 = float(bigFloat);
            }

            QCBORDecode_GetInt64(&m_cborDecoderCtx, &BigInteger);
            cmd.cmd_id = (int)BigInteger;
        }
        
        cmd_list.push_back(cmd);

        QCBORDecode_ExitMap(&m_cborDecoderCtx);
        QCBORDecode_Finish(&m_cborDecoderCtx);

    }
}

bool CborStreamStateMachine::CborStreamState_decode::get_cmd(cmd_t &cmd)
{
    if( !cmd_list.empty() )
    {
        cmd = cmd_list.front();
        cmd_list.pop_front();
        return true;
    }
    return false;
}
