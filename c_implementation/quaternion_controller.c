#include <math.h>
#include "quaternion_controller.h"

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

static void normalize_quaternion(double quaternion[4])
{
    double magnitude = sqrt(
        quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3]
    );

    if (magnitude <= 1.0e-15)
    {
        quaternion[0] = 1.0;
        quaternion[1] = 0.0;
        quaternion[2] = 0.0;
        quaternion[3] = 0.0;
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        quaternion[i] /= magnitude;
    }
}

void compute_quaternion_control_torque(
    const double command_quaternion[4],
    const double current_quaternion[4],
    const double body_rate[3],
    const double Kp[3],
    const double Kd[3],
    const double torque_limit[3],
    double control_torque[3],
    double error_quaternion[4]
)
{
    double command[4] = {
        command_quaternion[0],
        command_quaternion[1],
        command_quaternion[2],
        command_quaternion[3]
    };

    double current[4] = {
        current_quaternion[0],
        current_quaternion[1],
        current_quaternion[2],
        current_quaternion[3]
    };

    normalize_quaternion(command);
    normalize_quaternion(current);

    error_quaternion[0] =
        current[0] * command[0]
        + current[1] * command[1]
        + current[2] * command[2]
        + current[3] * command[3];

    error_quaternion[1] =
        current[0] * command[1]
        - current[1] * command[0]
        - current[2] * command[3]
        + current[3] * command[2];

    error_quaternion[2] =
        current[0] * command[2]
        + current[1] * command[3]
        - current[2] * command[0]
        - current[3] * command[1];

    error_quaternion[3] =
        current[0] * command[3]
        - current[1] * command[2]
        + current[2] * command[1]
        - current[3] * command[0];

    if (error_quaternion[0] < 0.0)
    {
        for (int i = 0; i < 4; i++)
        {
            error_quaternion[i] =
                -error_quaternion[i];
        }
    }

    normalize_quaternion(error_quaternion);

    for (int axis = 0; axis < 3; axis++)
    {
        double unsaturated_torque =
            2.0 * Kp[axis]
            * error_quaternion[axis + 1]
            - Kd[axis] * body_rate[axis];

        control_torque[axis] = clamp_value(
            unsaturated_torque,
            -torque_limit[axis],
            torque_limit[axis]
        );
    }
}