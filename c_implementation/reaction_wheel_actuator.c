#include "reaction_wheel_actuator.h"

static double clamp_value(
    double value,
    double minimum,
    double maximum
)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

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
)
{
    for (int axis = 0; axis < 3; axis++)
    {
        /*
         * First apply the wheel motor-torque limit to the requested
         * spacecraft torque.
         */
        applied_spacecraft_torque[axis] =
            clamp_value(
                requested_spacecraft_torque[axis],
                -wheel_torque_limit[axis],
                wheel_torque_limit[axis]
            );

        /*
         * The wheel motor torque is equal and opposite to the
         * spacecraft torque.
         */
        wheel_motor_torque[axis] =
            -applied_spacecraft_torque[axis];

        speed_limited[axis] = 0;

        /*
         * Prevent a wheel at its positive speed limit from receiving
         * additional positive wheel torque.
         */
        if (
            wheel_speed[axis] >= wheel_speed_limit[axis]
            && wheel_motor_torque[axis] > 0.0
        )
        {
            wheel_motor_torque[axis] = 0.0;
            speed_limited[axis] = 1;
        }

        /*
         * Prevent a wheel at its negative speed limit from receiving
         * additional negative wheel torque.
         */
        if (
            wheel_speed[axis] <= -wheel_speed_limit[axis]
            && wheel_motor_torque[axis] < 0.0
        )
        {
            wheel_motor_torque[axis] = 0.0;
            speed_limited[axis] = 1;
        }

        applied_spacecraft_torque[axis] =
            -wheel_motor_torque[axis];

        if (wheel_inertia[axis] > 0.0)
        {
            wheel_acceleration[axis] =
                wheel_motor_torque[axis]
                / wheel_inertia[axis];
        }
        else
        {
            wheel_acceleration[axis] = 0.0;
        }
    }
}