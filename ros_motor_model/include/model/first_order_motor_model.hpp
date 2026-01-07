#ifndef FIRST_ORDER_MOTOR_MODEL_HPP
#define FIRST_ORDER_MOTOR_MODEL_HPP

#include "ode_solver/ode_rk4_solver.hpp"

struct FirstOrderMotorParams
{
    double timeConstUp{0.01};
    double timeConstDown{0.01};
};

class FirstOrderMotorModel
{
    public:

    FirstOrderMotorModel();
    FirstOrderMotorModel(const FirstOrderMotorParams& params);

    void set_initial_time(const double &t_init);

    void set_state(const double &rps_cmd,
                   const double &time_curr);

    void get_state(double &rps_out);

    private:

    void compute_motor_dynamics(const double &rps_state,
                                double &rps_dot,
                                const double &t_prev);

    OdeRk4Solver<double> *rk4_solver_;

    FirstOrderMotorParams params_;

    double t_prev_{0};
    double t_curr_{0};

    double rps_cmd_{0};
    double rps_state_{0};

};


#endif // FIRST_ORDER_MOTOR_MODEL_H