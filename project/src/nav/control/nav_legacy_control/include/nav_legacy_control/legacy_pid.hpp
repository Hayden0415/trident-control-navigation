#pragma once
// updateAxis is taken from the legacy bot_sim velocity PID controller.
#include <algorithm>
#include <cmath>
struct LegacyPid {
    struct AxisState {
        double kp=0.35, ki=0.0, kd=0.02, integral=0, previous_error=0;
        bool has_previous_error=false;
    };
    double feedforward_=1.0, integral_limit_=0.5, deadband_=0.03;
    AxisState x, y;
    static double clamp(double v,double limit) {return std::clamp(v,-limit,limit);}
    void reset() { x.integral=y.integral=0; x.has_previous_error=y.has_previous_error=false; }
    double updateAxis(AxisState& axis, double desired, double measured, double dt) {
        // Compute the raw tracking error (full precision, not deadband-zeroed).
        const double raw_error = desired - measured;

        // Apply deadband to P and I: prevents fighting measurement noise when
        // the robot is already near the velocity setpoint.
        // The derivative continues to use raw_error (see below).
        const double control_error = std::abs(raw_error) < deadband_ ? 0.0 : raw_error;

        // Integrate control_error (deadband-zeroed) with anti-windup clamp.
        axis.integral = clamp(axis.integral + control_error * dt, integral_limit_);

        // Derivative term — only valid from the second cycle onward.
        double derivative = 0.0;
        if (axis.has_previous_error) {
            // Use raw_error (not control_error) so the derivative is continuous
            // when the error crosses the deadband boundary.
            derivative = (raw_error - axis.previous_error) / dt;

            // Both current and previous errors inside the deadband means the
            // robot is already well-controlled and the position noise floor
            // dominates.  Zero the derivative to avoid amplifying that noise.
            if (std::abs(raw_error) < deadband_ && std::abs(axis.previous_error) < deadband_) {
                derivative = 0.0;
            }
        }

        // Store raw_error so the next cycle's derivative is smooth across the
        // deadband boundary.
        axis.previous_error = raw_error;
        axis.has_previous_error = true;

        // Feedforward baseline (planner command) + PID correction.
        return feedforward_ * desired
             + axis.kp * control_error
             + axis.ki * axis.integral
             + axis.kd * derivative;
    }

};
