#ifndef QUATERNION_CONTROLLER_H
#define QUATERNION_CONTROLLER_H

void compute_quaternion_control_torque(
    const double command_quaternion[4],
    const double current_quaternion[4],
    const double body_rate[3],
    const double Kp[3],
    const double Kd[3],
    const double torque_limit[3],
    double control_torque[3],
    double error_quaternion[4]
);

#endif