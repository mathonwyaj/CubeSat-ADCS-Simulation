#include <math.h>
#include <stdio.h>

#include "quaternion_controller.h"
#include "reaction_wheel_actuator.h"

static int check_close(
    const char *name,
    double actual,
    double expected,
    double tolerance
)
{
    if (fabs(actual - expected) > tolerance)
    {
        printf(
            "FAIL: %s: actual = %.12f, expected = %.12f\n",
            name,
            actual,
            expected
        );

        return 0;
    }

    return 1;
}

static int check_integer(
    const char *name,
    int actual,
    int expected
)
{
    if (actual != expected)
    {
        printf(
            "FAIL: %s: actual = %d, expected = %d\n",
            name,
            actual,
            expected
        );

        return 0;
    }

    return 1;
}

int main(void)
{
    const double pi = 3.14159265358979323846;
    const double half_roll_angle = 15.0 * pi / 180.0;

    const double command_quaternion[4] = {
        1.0,
        0.0,
        0.0,
        0.0
    };

    const double current_quaternion[4] = {
        cos(half_roll_angle),
        sin(half_roll_angle),
        0.0,
        0.0
    };

    const double body_rate[3] = {
        0.0,
        0.0,
        0.0
    };

    const double Kp[3] = {
        0.04355,
        0.03350,
        0.01675
    };

    const double Kd[3] = {
        0.06955,
        0.05350,
        0.02675
    };

    const double wheel_inertia[3] = {
        2.0e-5,
        2.0e-5,
        2.0e-5
    };

    const double torque_limit[3] = {
        0.002,
        0.002,
        0.002
    };

    const double speed_limit[3] = {
        628.3185307179587,
        628.3185307179587,
        628.3185307179587
    };

    double requested_torque[3];
    double error_quaternion[4];

    int passed = 1;

    /*
     * Convert the 30-degree roll error into a controller torque.
     */
    compute_quaternion_control_torque(
        command_quaternion,
        current_quaternion,
        body_rate,
        Kp,
        Kd,
        torque_limit,
        requested_torque,
        error_quaternion
    );

    passed &= check_close(
        "requested roll torque",
        requested_torque[0],
        -0.002,
        1.0e-12
    );

    /*
     * Test the normal controller-to-wheel command chain.
     */
    {
        const double wheel_speed[3] = {
            0.0,
            0.0,
            0.0
        };

        double spacecraft_torque[3];
        double wheel_acceleration[3];
        double motor_torque[3];
        int speed_limited[3];

        compute_reaction_wheel_actuation(
            requested_torque,
            wheel_speed,
            wheel_inertia,
            torque_limit,
            speed_limit,
            spacecraft_torque,
            wheel_acceleration,
            motor_torque,
            speed_limited
        );

        passed &= check_close(
            "applied spacecraft roll torque",
            spacecraft_torque[0],
            -0.002,
            1.0e-12
        );

        passed &= check_close(
            "wheel motor roll torque",
            motor_torque[0],
            0.002,
            1.0e-12
        );

        passed &= check_close(
            "wheel roll acceleration",
            wheel_acceleration[0],
            100.0,
            1.0e-9
        );

        passed &= check_integer(
            "normal speed-limit flag",
            speed_limited[0],
            0
        );
    }

    /*
     * At positive maximum wheel speed, positive motor torque must
     * be blocked, causing loss of roll control authority.
     */
    {
        const double wheel_speed[3] = {
            speed_limit[0],
            0.0,
            0.0
        };

        double spacecraft_torque[3];
        double wheel_acceleration[3];
        double motor_torque[3];
        int speed_limited[3];

        compute_reaction_wheel_actuation(
            requested_torque,
            wheel_speed,
            wheel_inertia,
            torque_limit,
            speed_limit,
            spacecraft_torque,
            wheel_acceleration,
            motor_torque,
            speed_limited
        );

        passed &= check_close(
            "saturated spacecraft roll torque",
            spacecraft_torque[0],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "saturated wheel acceleration",
            wheel_acceleration[0],
            0.0,
            1.0e-12
        );

        passed &= check_integer(
            "saturated speed-limit flag",
            speed_limited[0],
            1
        );
    }

    if (!passed)
    {
        printf("ADCS command-chain tests FAILED.\n");
        return 1;
    }

    printf("ADCS command-chain tests PASSED.\n");
    return 0;
}