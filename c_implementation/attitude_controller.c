#include "attitude_controller.h"

double compute_control_torque(
    double theta_cmd,
    double theta,
    double omega,
    double Kp,
    double Kd,
    double tau_max
)
{
    double error = theta_cmd - theta;

    double torque_cmd = Kp * error - Kd * omega;

    if (torque_cmd > tau_max)
    {
        return tau_max;
    }
    else if (torque_cmd < -tau_max)
    {
        return -tau_max;
    }

    return torque_cmd;
}

