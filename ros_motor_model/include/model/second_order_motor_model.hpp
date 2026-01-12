#ifndef SECOND_ORDER_MOTOR_MODEL_HPP
#define SECOND_ORDER_MOTOR_MODEL_HPP

#include "ode_solver/ode_rk4_solver.hpp"
#include "utils/type_def.h"
#include <algorithm>

struct SecondOrderMotorParams
{
    double p1{25.16687};    // Friction coefficient
    double p2{0.003933};     // Drag coefficient
    double p3{515.605};   // Motor Stiffness
    double jerk_max{250000.0};      // Maximum angular jerk [RPM/s^2]
    double alpha_max{15000.0};      // Maximum angular acceleration [RPM/s]
};


class SecondOrderMotorModel
{
    public:

    SecondOrderMotorModel();
    SecondOrderMotorModel(const SecondOrderMotorParams& params);

    void set_initial_time(const double &t_init);

    void set_state(const double &rpm_cmd,
                   const double &time_curr);

    void get_state(double &rpm_out);

    private:

    void compute_motor_dynamics(const Vector2d &state,
                                Vector2d &dot_state,
                                const double &t_prev);

    OdeRk4Solver<Vector2d> *rk4_solver_;

    SecondOrderMotorParams params_;

    double t_prev_{0.0};
    double t_curr_{0.0};

    double rpm_cmd_{0.0};
    Vector2d state_;
};


#endif // SECOND_ORDER_MOTOR_MODEL_HPP