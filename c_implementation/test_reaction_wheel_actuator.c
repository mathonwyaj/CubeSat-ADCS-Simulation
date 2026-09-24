#include <math.h>
#include <stdio.h>
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

    int passed = 1;

    /*
     * Test 1: motor-torque saturation.
     */
    {
        const double requested_torque[3] = {
            0.003,
            -0.001,
            0.0
        };

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
            "saturated spacecraft torque x",
            spacecraft_torque[0],
            0.002,
            1.0e-12
        );

        passed &= check_close(
            "spacecraft torque y",
            spacecraft_torque[1],
            -0.001,
            1.0e-12
        );

        passed &= check_close(
            "wheel acceleration x",
            wheel_acceleration[0],
            -100.0,
            1.0e-9
        );

        passed &= check_close(
            "wheel acceleration y",
            wheel_acceleration[1],
            50.0,
            1.0e-9
        );

        passed &= check_integer(
            "torque-test speed flag x",
            speed_limited[0],
            0
        );
    }

    /*
     * Test 2: wheel-speed saturation.
     */
    {
        const double requested_torque[3] = {
            -0.001,
            0.001,
            0.001
        };

        const double wheel_speed[3] = {
            speed_limit[0],
            -speed_limit[1],
            speed_limit[2]
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
            "speed-limited spacecraft torque x",
            spacecraft_torque[0],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "speed-limited spacecraft torque y",
            spacecraft_torque[1],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "allowed spacecraft torque z",
            spacecraft_torque[2],
            0.001,
            1.0e-12
        );

        passed &= check_close(
            "allowed wheel acceleration z",
            wheel_acceleration[2],
            -50.0,
            1.0e-9
        );

        passed &= check_integer(
            "speed flag x",
            speed_limited[0],
            1
        );

        passed &= check_integer(
            "speed flag y",
            speed_limited[1],
            1
        );

        passed &= check_integer(
            "speed flag z",
            speed_limited[2],
            0
        );
    }

    if (!passed)
    {
        printf("Reaction-wheel actuator tests FAILED.\n");
        return 1;
    }

    printf("Reaction-wheel actuator tests PASSED.\n");
    return 0;
}