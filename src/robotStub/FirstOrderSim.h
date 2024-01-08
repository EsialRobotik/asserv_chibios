#ifndef FIRST_ORDER_SIM_H_
#define FIRST_ORDER_SIM_H_


class FirstOrderSim
{
    public:
    FirstOrderSim(float period, float tau, float static_gain);

        float process(float input);

private:
       float E, ///< Constant, depending on sampling rate and time constant
             static_gain_,
             last_output_;
};


#endif /* MOTOR_CONTROLLER_SIMULATION_H_ */
