#include <math.h>
#include <stdio.h>
#include "quaternion_controller.h"

static int check_close(
    const char *name,
    double actual,
    double expected,
    double tolerance
)
{
    double error = fabs(actual - expected);

    if (error > tolerance)
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

int main(void)
{
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

    const double torque_limit[3] = {
        0.002,
        0.002,
        0.002
    };

    const double command_quaternion[4] = {
        1.0,
        0.0,
        0.0,
        0.0
    };

    int passed = 1;

    /*
     * Test 1: identity attitude with zero body rate.
     */
    {
        const double current_quaternion[4] = {
            1.0,
            0.0,
            0.0,
            0.0
        };

        const double body_rate[3] = {
            0.0,
            0.0,
            0.0
        };

        double torque[3];
        double error_quaternion[4];

        compute_quaternion_control_torque(
            command_quaternion,
            current_quaternion,
            body_rate,
            Kp,
            Kd,
            torque_limit,
            torque,
            error_quaternion
        );

        passed &= check_close(
            "identity torque x",
            torque[0],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "identity torque y",
            torque[1],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "identity torque z",
            torque[2],
            0.0,
            1.0e-12
        );
    }

    /*
     * Test 2: current attitude is positive 30-degree roll.
     * The controller must command negative saturated x torque.
     */
    {
        const double half_angle = 15.0 * 3.141592653589793 / 180.0;

        const double current_quaternion[4] = {
            cos(half_angle),
            sin(half_angle),
            0.0,
            0.0
        };

        const double body_rate[3] = {
            0.0,
            0.0,
            0.0
        };

        double torque[3];
        double error_quaternion[4];

        compute_quaternion_control_torque(
            command_quaternion,
            current_quaternion,
            body_rate,
            Kp,
            Kd,
            torque_limit,
            torque,
            error_quaternion
        );

        passed &= check_close(
            "roll error scalar",
            error_quaternion[0],
            cos(half_angle),
            1.0e-12
        );

        passed &= check_close(
            "roll error x",
            error_quaternion[1],
            -sin(half_angle),
            1.0e-12
        );

        passed &= check_close(
            "roll torque x",
            torque[0],
            -0.002,
            1.0e-12
        );

        passed &= check_close(
            "roll torque y",
            torque[1],
            0.0,
            1.0e-12
        );

        passed &= check_close(
            "roll torque z",
            torque[2],
            0.0,
            1.0e-12
        );
    }

    /*
     * Test 3: q and -q must produce the same torque.
     */
    {
        const double half_angle = 15.0 * 3.141592653589793 / 180.0;

        const double negative_quaternion[4] = {
            -cos(half_angle),
            -sin(half_angle),
            0.0,
            0.0
        };

        const double body_rate[3] = {
            0.0,
            0.0,
            0.0
        };

        double torque[3];
        double error_quaternion[4];

        compute_quaternion_control_torque(
            command_quaternion,
            negative_quaternion,
            body_rate,
            Kp,
            Kd,
            torque_limit,
            torque,
            error_quaternion
        );

        passed &= check_close(
            "negative quaternion torque x",
            torque[0],
            -0.002,
            1.0e-12
        );
    }

    if (!passed)
    {
        printf("Quaternion controller tests FAILED.\n");
        return 1;
    }

    printf("Quaternion controller tests PASSED.\n");
    return 0;
}