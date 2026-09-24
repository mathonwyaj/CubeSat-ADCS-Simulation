#include <stdio.h>
#include "attitude_controller.h"

int main(void)
{
    const double pi = 3.14159265358979323846;

    double Kp = 0.0067;
    double Kd = 0.0107;
    double tau_max = 0.002;

    double theta_cmd = 0.0;

    double theta_cases[] = {
        30.0 * pi / 180.0,
        10.0 * pi / 180.0,
        1.0 * pi / 180.0,
        -5.0 * pi / 180.0
    };

    double omega_cases[] = {
        0.0,
        -0.10,
        -0.02,
        0.05
    };

    int num_cases = 4;

    for (int i = 0; i < num_cases; i++)
    {
        double torque = compute_control_torque(
            theta_cmd,
            theta_cases[i],
            omega_cases[i],
            Kp,
            Kd,
            tau_max
        );

        printf(
            "Case %d: theta = %.4f rad, "
            "omega = %.4f rad/s, "
            "torque = %.6f N*m\n",
            i + 1,
            theta_cases[i],
            omega_cases[i],
            torque
        );
    }

    return 0;
}