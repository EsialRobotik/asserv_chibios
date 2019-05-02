#include "Encoders.h"
#include "ch.h"
#include "hal.h"

static QEIConfig qeicfg1 = {
  QEI_MODE_QUADRATURE,
  QEI_BOTH_EDGES,
  QEI_DIRINV_FALSE,
};

static QEIConfig qeicfg2 = {
  QEI_MODE_QUADRATURE,
  QEI_BOTH_EDGES,
  QEI_DIRINV_FALSE,
};

Encoders::Encoders()
{
}

Encoders::~Encoders()
{
}

void Encoders::init()
{
	// Encoder 1
	palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
	qeiStart(&QEID3, &qeicfg1);

	// Encoder 2
	palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
	palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
	qeiStart(&QEID2, &qeicfg2);
}

void Encoders::start()
{
	qeiEnable (&QEID3);
	qeiEnable (&QEID2);
}

void Encoders::stop()
{
	qeiDisable(&QEID3);
	qeiDisable(&QEID2);
}
void Encoders::getValues(int16_t *encoder1, int16_t *encoder2)
{
	*encoder1 = qeiGetCount(&QEID2);
	*encoder2 = qeiGetCount(&QEID3);
}

void Encoders::getValuesAndReset(int16_t *encoder1, int16_t *encoder2)
{
	getValues(encoder1, encoder2);
	qei_lld_set_count(&QEID2, 0);
	qei_lld_set_count(&QEID3, 0);
}
