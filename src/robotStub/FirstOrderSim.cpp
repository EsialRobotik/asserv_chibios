#include "FirstOrderSim.h"
#include <cmath>


FirstOrderSim::FirstOrderSim(float period, float tau, float static_gain)
{
    E = expf(-period / tau);
    last_output_ = 0;
    static_gain_ = static_gain;
}

float FirstOrderSim::process(float input) {
    last_output_ = static_gain_ * input + (last_output_ - static_gain_ * input) * E;
    return last_output_;
}


void FirstOrderSim::reset()
{
	last_output_ = 0;
}
