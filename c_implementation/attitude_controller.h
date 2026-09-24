#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

double compute_control_torque(
    double theta_cmd,
    double theta,
    double omega,
    double Kp,
    double Kd,
    double tau_max
);

#endif
