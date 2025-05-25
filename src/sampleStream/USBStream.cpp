#include "USBStream.h"
#include "usbcfg.h"
#include <ch.h>
#include <hal.h>
#include "core_cm4.h"
#include <cstring>

const uint32_t synchroWord_stream = 0xCAFED00D;
const uint32_t synchroWord_config = 0xCAFEDECA;
const uint32_t synchroWord_connection = 0xDEADBEEF;

USBStream::USBStream(uint16_t loopFrequency)
{
    m_timestamp = 0;
    m_loopFrequency = loopFrequency;
    chMtxObjectInit (&m_sample_sending_mutex);
    std::memset(m_currentStruct.array, 0xFFFFFFFF, sizeof(m_currentStruct.array));
}

void USBStream::init(UsbStreamPinConf_t *pinConf, uint16_t loopFrequency)
{
    // Init pin if needed
    if(pinConf)
    {
        palSetPadMode(pinConf->dataPlusPin_GPIObase, pinConf->dataPlusPin_number, PAL_MODE_ALTERNATE(pinConf->dataPlusPin_alternate)); //USB D+
        palSetPadMode(pinConf->dataMinusPin_GPIObase, pinConf->dataMinusPin_number, PAL_MODE_ALTERNATE(pinConf->dataMinusPin_alternate)); //USB D-
    }
//    palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10)); //USB D+
//    palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10)); //USB D-


    s_instance = new USBStream(loopFrequency);
    SampleStream::setInstance(s_instance);

    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    std::memset(SDU1.ob, 0x0F0F0F0F, sizeof(SDU1.ob));
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    s_instance->getEmptyBuffer();

}

void* USBStream::sendCurrentStream()
{
    m_currentStruct.synchro = synchroWord_stream;
    m_currentStruct.sample_size_without_synchro_nor_this = sizeof(m_currentStruct)-sizeof(m_currentStruct.synchro)-sizeof(m_currentStruct.sample_size_without_synchro_nor_this);
    setTimestamp(m_timestamp++);

    if( chMtxTryLock(&m_sample_sending_mutex))
    {
        sendFullBuffer();

        getEmptyBuffer();
        chMtxUnlock(&m_sample_sending_mutex);
    }

    return m_currentPtr;
}

void USBStream::sendConfig(uint8_t *configBuffer, uint8_t size)
{
    chnWrite(&SDU1, (uint8_t*)&synchroWord_config, sizeof(synchroWord_config));
    chnWrite(&SDU1, &size, sizeof(size));
    chnWrite(&SDU1, configBuffer, size);
}

void USBStream::sendFullBuffer()
{
    if (m_currentPtr != NULL) {
        *m_currentPtr = m_currentStruct;
        obqPostFullBuffer(&SDU1.obqueue, sizeof(UsbStreamSample));
    }
}

void USBStream::getEmptyBuffer()
{
    msg_t msg = obqGetEmptyBufferTimeout(&SDU1.obqueue, 0);
    if (msg == MSG_OK) {
        m_currentPtr = (UsbStreamSample*) SDU1.obqueue.ptr;
        uint32_t available_size = ((uint32_t) SDU1.obqueue.top - (uint32_t) SDU1.obqueue.ptr);
        chDbgAssert(available_size >= (sizeof(UsbStreamSample) + 4),
                "Not enough space in the free buffer. Did you set a correct USB buffer size ?");
    } else {
        m_currentPtr = NULL;
    }
}

void USBStream::getFullBuffer(void **ptr, uint32_t *size)
{
    *size = 0;
    if (!is_usb_serial_configured()) {
        /*
         *  Workaround :
         *   When USB isn't plugged, ibqGetFullBufferTimeout return MSG_RESET immediately and this thread stuck others threads.
         *   is_usb_serial_configured return the number of time the usb serial driver was configured ( ie : plugged in a PC ),
         *    so while this condition isn't true, just schedule the others thread.
         */

        chThdYield();
    }

    msg_t msg = ibqGetFullBufferTimeout(&SDU1.ibqueue, TIME_INFINITE);

    uint32_t sizeToRead = (size_t) SDU1.ibqueue.top - (size_t) SDU1.ibqueue.ptr;
    if (msg != MSG_OK || sizeToRead == 0)
        return;

    *size = sizeToRead;
    *ptr = SDU1.ibqueue.ptr;
}

void USBStream::releaseBuffer()
{
    ibqReleaseEmptyBuffer(&SDU1.ibqueue);
}

extern BaseSequentialStream *outputStream;
void USBStream::USBStreamHandleConnection_lowerpriothread(usbStreamCallback callback)
{
    void *ptr = nullptr;
    uint32_t size = 0;
    getFullBuffer(&ptr, &size);
    if (size > 0)
    {
        uint32_t *buffer = (uint32_t*) ptr;
        if(buffer[0] == synchroWord_connection)
        {
            chprintf(outputStream, "connection detected\r\n");
            /*
             * It's a bit tricky here !
             * As this function is paced by a medium priority thread ( ie: bellow than the rest of this class )
             *  sending a description must stop the sending of the asserv loop.
             *  To do so, a mutex is used and a new buffer is get for the next loop at the end of this routine.
             */
            chMtxLock(&m_sample_sending_mutex);

            obqGetEmptyBufferTimeout(&SDU1.obqueue, TIME_MS2I(100));
            uint32_t *ptr_32 = (uint32_t*)SDU1.obqueue.ptr;

             //The description begin with a synchro word //
            ptr_32[0] = synchroWord_connection;

            // ... then the size of the following string plus the size of the frequency loop at the end !
            // The value will be inserted at the end of this procedure because the frequency loop string isn't computed yet 
            uint32_t description_size = strlen(description);

            // And finaly, the description itself
            char *ptr_str = (char*)&(ptr_32[2]);
            unsigned int i;
            for(i=0;i<description_size; i++)
                ptr_str[i] = description[i];

            // But wait, there's more ! At the end send the loop frequency and update the whole descripition size
            int len = sprintf(&ptr_str[i], "freq=%d", m_loopFrequency);
            ptr_32[1]+= len;
            description_size += len;
            
            // Insert the size of whole descripition
            ptr_32[1] = description_size;

            obqPostFullBuffer(&SDU1.obqueue, ((uint8_t*)&ptr_str[description_size]) - ((uint8_t*)ptr_32));

            getEmptyBuffer();
            chMtxUnlock(&m_sample_sending_mutex);
        }
        else
        {
            char *str = (char*) ptr;
            str[size] = 0;
            callback(str, size);
        }
        releaseBuffer();
    }
}

