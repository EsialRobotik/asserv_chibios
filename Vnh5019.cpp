#include "Vnh5019.h"
#include "ch.h"
#include "hal.h"

static PWMConfig pwmcfg = {
   200000,
   1000,
   NULL,
   {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
   },
   0,
   0
};

static PWMConfig pwmcfg2 = {
   200000,
   1000,
   NULL,
   {
	  {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
   },
   0,
   0
};

Vnh5019::Vnh5019(bool invertMotor1, bool invertMotor2)
{
	m_invertMotor1 = invertMotor1;
	m_invertMotor2 = invertMotor2;
}
void Vnh5019::init()
{
	// MOTOR 1 control
	palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);

	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(3)); //TIM4_chan1
	pwmStart(&PWMD8, &pwmcfg);

	// MOTOR 2 control
	palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);


	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2)); //TIM4_chan1
	pwmStart(&PWMD4, &pwmcfg2);
}

void Vnh5019::setMotor1Speed(float percentage)
{
	if(m_invertMotor1)
		percentage = -percentage;

	unsigned char reverse = 0;
	if (percentage < 0)
	{
		percentage = -percentage;
		reverse = 1;
	}

	if (percentage > 100.0)
		percentage = 100.0;

	if (percentage == 0)
	{
		palClearPad(GPIOA, 10); //M1INA
		palClearPad(GPIOB, 5); 	//M1INB
	}
	else if (reverse)
	{
		palClearPad(GPIOA, 10); //M1INA
		palSetPad(GPIOB, 5); 	//M1INB
	}
	else
	{
		palSetPad(GPIOA, 10); //M1INA
		palClearPad(GPIOB, 5); 	//M1INB
	}

	pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, (unsigned int)(10000*percentage/100.0))); // 10000 is 100% duty cycle
}

void Vnh5019::setMotor2Speed(float percentage)
{
	if(m_invertMotor2)
		percentage = -percentage;

	unsigned char reverse = 0;
	if (percentage < 0)
	{
		percentage = -percentage;
		reverse = 1;
	}

	if (percentage > 100.0)
		percentage = 100.0;

	if (percentage == 0)
	{
		palClearPad(GPIOA, 8); //M2INA
		palClearPad(GPIOA, 9); //M2INB
	}
	else if (reverse)
	{
		palClearPad(GPIOA, 8); //M2INA
		palSetPad(GPIOA, 9); //M2INB
	}
	else
	{
		palSetPad(GPIOA, 8); //M2INA
		palClearPad(GPIOA, 9); //M2INB
	}

	pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, (unsigned int)(10000*percentage/100.0))); // 10000 is 100% duty cycle
}


Vnh5019::~Vnh5019()
{
}

