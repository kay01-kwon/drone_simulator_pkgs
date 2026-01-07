#include "model/first_order_motor_model.hpp"

FirstOrderMotorModel::FirstOrderMotorModel()
{
    rk4_solver_ = new OdeRk4Solver<double>();
}

FirstOrderMotorModel::FirstOrderMotorModel(const FirstOrderMotorParams& params)
: params_(params)
{
    rk4_solver_ = new OdeRk4Solver<double>();
}

void FirstOrderMotorModel::set_initial_time(const double &t_init)
{
    t_curr_ = t_init;
    t_prev_ = t_init;
}

void FirstOrderMotorModel::set_state(const double &rps_cmd,
                           const double &time_curr)
{
    rps_cmd_ = rps_cmd;
    t_curr_ = time_curr;
    double dt = t_curr_ - t_prev_;

    // Run the rk4 solver to compute the new motor state
    rk4_solver_->do_step(
        [this](const double &rps_state,
                double &rps_dot,
                const double &t_prev)
        {
            compute_motor_dynamics(rps_state, rps_dot, t_prev);
        },
        rps_state_,
        t_prev_,
        dt
    );

    
    t_prev_ = t_curr_;
}

void FirstOrderMotorModel::get_state(double &rps_out)
{
    rps_out = rps_state_;
}

void FirstOrderMotorModel::compute_motor_dynamics(const double &rps_state,
                                        double &rps_dot,
                                        const double &t_prev)
{
    double timeConst = (rps_cmd_ > rps_state) ? params_.timeConstUp : params_.timeConstDown;
    // First order motor dynamics: drps/dt = (rps_cmd - rps_state) / timeConst
    rps_dot = (rps_cmd_ - rps_state) / timeConst;
}