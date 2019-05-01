#ifndef USBSTREAMTYPE_SRC_DATASTREAMTYPE_H_
#define USBSTREAMTYPE_SRC_DATASTREAMTYPE_H_


typedef struct
{
	uint32_t synchro;
	uint32_t timestamp;
	float current_phaseA;
	float current_phaseB;
	float current_phaseC;
	uint8_t tempPhaseA;
	uint8_t tempPhaseB;
	uint8_t tempPhaseC;
	uint8_t vBat;
}  __attribute__((packed)) UsbStreamSample;



#endif /* USBSTREAMTYPE_SRC_DATASTREAMTYPE_H_ */
