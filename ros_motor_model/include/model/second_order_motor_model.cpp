#include "second_order_motor_model.hpp"

SecondOrderMotorModel::SecondOrderMotorModel()
{
    rk4_solver_ = new OdeRk4Solver<Vector2d>();
    state_.setZero();
    rpm_cmd_ = 0.0;
    delay_s_ = 0.0;
}

SecondOrderMotorModel::SecondOrderMotorModel(const SecondOrderMotorParams& params)
: params_(params)
{
    rk4_solver_ = new OdeRk4Solver<Vector2d>();
    state_.setZero();
    rpm_cmd_ = 0.0;
    delay_s_ = params_.delay_ms / 1000.0;
}

void SecondOrderMotorModel::set_initial_time(const double &t_init)
{
    t_curr_ = t_init;
    t_prev_ = t_init;
}

void SecondOrderMotorModel::set_state(const double &rpm_cmd,
                               const double &time_curr)
{
    t_curr_ = time_curr;
    double dt = t_curr_ - t_prev_;

    if (delay_s_ > 0.0) {
        cmd_history_.emplace_back(time_curr, rpm_cmd);

        double t_cutoff = time_curr - delay_s_ * 2.0;
        while (cmd_history_.size() > 2 && cmd_history_.front().first < t_cutoff) {
            cmd_history_.pop_front();
        }

        rpm_cmd_ = get_delayed_cmd(time_curr);
    } else {
        rpm_cmd_ = rpm_cmd;
    }

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

    if (!params_.pure) {
        state_(1) = std::clamp(state_(1), -params_.alpha_max, params_.alpha_max);
    }

    t_prev_ = t_curr_;
}

void SecondOrderMotorModel::get_state(double &rpm_out)
{
    rpm_out = state_(0);
}

double SecondOrderMotorModel::get_delayed_cmd(const double &time_curr)
{
    double t_target = time_curr - delay_s_;

    if (cmd_history_.empty()) {
        return 0.0;
    }

    if (t_target <= cmd_history_.front().first) {
        return cmd_history_.front().second;
    }

    for (size_t i = 1; i < cmd_history_.size(); ++i) {
        if (cmd_history_[i].first >= t_target) {
            double t0 = cmd_history_[i - 1].first;
            double t1 = cmd_history_[i].first;
            double v0 = cmd_history_[i - 1].second;
            double v1 = cmd_history_[i].second;
            double alpha = (t_target - t0) / (t1 - t0);
            return v0 + alpha * (v1 - v0);
        }
    }

    return cmd_history_.back().second;
}

void SecondOrderMotorModel::compute_motor_dynamics(const Vector2d &state,
                                        Vector2d &dot_state,
                                        const double &t_prev)
{

    double w = state(0);
    double w_dot = state(1);

    if (params_.pure) {
        double wn2 = params_.omega_n * params_.omega_n;
        double w_ddot = wn2 * (rpm_cmd_ - w) - 2.0 * params_.zeta * params_.omega_n * w_dot;
        dot_state(0) = w_dot;
        dot_state(1) = w_ddot;
        return;
    }

    double w_dot_eff, j_eff;

    double j;
    j = -(params_.p1 + params_.p2 * w)*w_dot
        - params_.p3 *(w - rpm_cmd_);

    j = std::clamp(j, -params_.jerk_max, params_.jerk_max);


    if (w_dot > params_.alpha_max && j > 0)
    {
        w_dot_eff = params_.alpha_max;
        j_eff = 0.0;
    }
    else if (w_dot < -params_.alpha_max && j < 0)
    {
        w_dot_eff = -params_.alpha_max;
        j_eff = 0.0;
    }
    else
    {
        w_dot_eff = w_dot;
        j_eff = j;
    }
    dot_state(0) = w_dot_eff;
    dot_state(1) = j_eff;
}