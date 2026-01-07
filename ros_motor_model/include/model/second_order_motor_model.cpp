#include "second_order_motor_model.hpp"

SecondOrderMotorModel::SecondOrderMotorModel()
{
    rk4_solver_ = new OdeRk4Solver<Vector2d>();
    state_.setZero();
    rpm_cmd_ = 0.0;
}

SecondOrderMotorModel::SecondOrderMotorModel(const SecondOrderMotorParams& params)
: params_(params)
{
    rk4_solver_ = new OdeRk4Solver<Vector2d>();
    state_.setZero();
    rpm_cmd_ = 0.0;
}

void SecondOrderMotorModel::set_initial_time(const double &t_init)
{
    t_curr_ = t_init;
    t_prev_ = t_init;
}

void SecondOrderMotorModel::set_state(const double &rpm_cmd,
                               const double &time_curr)
{
    rpm_cmd_ = rpm_cmd;
    t_curr_ = time_curr;
    double dt = t_curr_ - t_prev_;

    // Run the rk4 solver to compute the new motor state
    rk4_solver_->do_step(
        [this](const Vector2d &motor_state,
                Vector2d &motor_dot_state,
                const double &t_prev)
        {
            compute_motor_dynamics(motor_state, motor_dot_state, t_prev);
        },
        state_,
        t_prev_,
        dt
    );

    
    t_prev_ = t_curr_;
}

void SecondOrderMotorModel::get_state(double &rpm_out)
{
    rpm_out = state_(0);
}

void SecondOrderMotorModel::compute_motor_dynamics(const Vector2d &state,
                                        Vector2d &dot_state,
                                        const double &t_prev)
{

    double w = state(0); 
    double w_dot = state(1);

    double w_dot_eff, j_eff;

    double j;
    j = -(params_.p1 + params_.p2 * w)*w_dot 
        - params_.p3 *(w - rpm_cmd_);

    j = std::clamp(j, -params_.jerk_max, params_.jerk_max);


    if (w_dot > params_.alpha_max && j > 0)
    {
        // Saturate angular acceleration (Positive direction)
        w_dot_eff = params_.alpha_max;
        j_eff = 0.0;
    }
    else if (w_dot < -params_.alpha_max && j < 0)
    {
        // Saturate angular acceleration (Negative direction)
        w_dot_eff = -params_.alpha_max;
        j_eff = 0.0;
    }
    else
    {
        // No saturation needed --> just apply computed values
        w_dot_eff = w_dot;
        j_eff = j;
    }
    dot_state(0) = w_dot_eff;
    dot_state(1) = j_eff;
}