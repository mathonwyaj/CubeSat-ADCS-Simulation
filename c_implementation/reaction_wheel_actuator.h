#ifndef REACTION_WHEEL_ACTUATOR_H
#define REACTION_WHEEL_ACTUATOR_H

void compute_reaction_wheel_actuation(
    const double requested_spacecraft_torque[3],
    const double wheel_speed[3],
    const double wheel_inertia[3],
    const double wheel_torque_limit[3],
    const double wheel_speed_limit[3],
    double applied_spacecraft_torque[3],
    double wheel_acceleration[3],
    double wheel_motor_torque[3],
    int speed_limited[3]
);

#endif