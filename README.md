# CubeSat Attitude Determination, Control, and Orbital Dynamics

## Overview

This project implements and validates a CubeSat attitude-determination and control system (ADCS) together with Low Earth Orbit propagation. It progressed from a single-axis PD baseline to a three-axis quaternion model with reaction wheels, sensor simulation, multiplicative extended Kalman filtering (MEKF), environmental disturbances, momentum dumping, Monte Carlo verification, Simulink integration, and a tested C implementation.

The current design includes:

- A 4 kg asymmetric 0.1 m x 0.2 m x 0.3 m CubeSat
- Quaternion rigid-body dynamics and three-axis PD control
- Three orthogonal reaction wheels
- Gyroscope and star-tracker models
- An MEKF estimating attitude and gyro bias
- Gravity-gradient, aerodynamic, solar-radiation-pressure, and magnetic disturbances
- Magnetorquer momentum dumping
- Two-body, J2, and simplified atmospheric-drag orbit models
- MATLAB, Simulink, and C implementations with automated tests

## Baseline Single-Axis Model

The original `cubesat_attitude.slx` model is retained as the baseline. It uses `Kp = 0.0067`, `Kd = 0.0107`, a +/-0.002 N m torque limit, and a 30 deg initial error.

| Metric | Result |
|---|---:|
| Final pointing error | 0.0000 deg |
| Maximum angular velocity | 12.6173 deg/s |
| Maximum torque | 0.002000 N m |
| Settling time within 0.5 deg | 4.016 s |

## Three-Axis Quaternion ADCS

The asymmetric inertia is:

```text
Ixx = 0.0433333 kg m^2
Iyy = 0.0333333 kg m^2
Izz = 0.0166667 kg m^2
```

Inertia-scaled controller gains are:

```text
Kp = [0.04355, 0.03350, 0.01675]
Kd = [0.06955, 0.05350, 0.02675]
```

For the combined initial attitude `[30, -20, 15] deg`, the tuned asymmetric quaternion model settled within 0.5 deg in 10.010 s with final error approximately zero and maximum quaternion norm error `9.379e-13`.

## Reaction Wheels

Final actuator design:

```text
Wheel inertia: 2.0e-5 kg m^2 per axis
Maximum torque: 0.002 N m per axis
Maximum speed: 6000 rpm per axis
Maximum momentum: 0.0126 N m s per axis
```

The wheel inertia was increased from the preliminary `1.0e-5 kg m^2` design after the asymmetric nominal case reached approximately 6016 rpm. The resized nominal design produced peak speeds of approximately `[3354.5, 1476.5, 1130.1] rpm` with no speed-limit activation.

The actuator model includes torque saturation, speed saturation, equal-and-opposite spacecraft/wheel torque, and loss of authority when a saturated wheel is commanded farther into saturation.

## Sensors and MEKF

```text
Gyroscope rate: 100 Hz
Gyroscope noise standard deviation: 0.020 deg/s
Initial gyro bias: [0.050, -0.030, 0.020] deg/s
Star-tracker rate: 5 Hz
Star-tracker noise standard deviation: 0.010 deg
```

Nominal estimated-state closed-loop results:

- Final control error: 0.0065 deg
- Final estimation error: 0.0031 deg
- RMS estimation error: 0.0146 deg
- Settling time: 9.92 s
- No wheel-speed-limit activation

## Environmental Disturbances

The model includes gravity-gradient, aerodynamic, solar-radiation-pressure, and residual magnetic-dipole torques. The 600 s combined test produced:

| Metric | Result |
|---|---:|
| Maximum pointing error | `1.287133e-4 deg` |
| RMS pointing error | `1.164074e-4 deg` |
| Maximum gravity-gradient torque | `1.790101e-8 N m` |
| Maximum aerodynamic torque | `2.552528e-8 N m` |
| Maximum SRP torque | `2.736000e-9 N m` |
| Maximum magnetic torque | `2.487239e-8 N m` |
| Maximum total disturbance | `4.335953e-8 N m` |

The maximum absolute wheel speeds were approximately `[0.000006, 7.125432, 9.024754] rpm`, with no speed-limit activation.

## Momentum Dumping

Magnetorquer momentum unloading was tested for two inclined orbits from `[3000, -2000, 1000] rpm`.

- Final wheel speeds: `[2.110, 0.458, 1.407] rpm`
- Initial wheel-momentum magnitude: `7.836509e-3 N m s`
- Final wheel-momentum magnitude: `5.397555e-6 N m s`
- Momentum reduction: 99.931%
- Maximum commanded dipole: `[0.168411, 0.200000, 0.114636] A m^2`
- Dipole limit: `0.2 A m^2` per axis
- Maximum pointing error: `0 deg`
- Wheel-speed-limit activations: none

The implementation accounts for the restriction that magnetic torque is perpendicular to the instantaneous magnetic field.

## Orbital Dynamics

The nominal 500 km orbit has a circular speed of approximately 7.617 km/s and period of 94.469 min.

### J2 validation

- Initial J2 acceleration: 0.1396% of central gravity
- Measured RAAN rate at 51.6 deg inclination: -4.779090 deg/day
- Theoretical RAAN rate: -4.758999 deg/day
- Three-day RAAN change: -14.357100 deg
- Two-body RAAN change: approximately zero

### Simplified drag comparison

For seven days with constant density `1.0e-12 kg/m^3`:

- J2-plus-drag semi-major-axis change: -293.954 m
- Drag-relative semi-major-axis difference: -304.886 m
- Final position difference: 153.719 km

This constant-density model is for controlled engineering comparison rather than high-fidelity lifetime prediction.

## Monte Carlo Verification

### Plant and actuator uncertainty

One hundred cases varied initial attitude, spacecraft inertia, wheel inertia, and available torque.

- Success rate: 100/100
- Worst final pointing error: 0.002259 deg
- Worst settling time: 11.920 s
- Maximum wheel speed: 4089.797 rpm
- Speed-limit cases: 0

### Full estimated-state closed loop

One hundred cases additionally varied gyro bias, gyro noise, bias random walk, star-tracker noise, and initial estimator error.

- Success rate: 100/100
- Worst final control error: 0.017219 deg
- Worst final estimation error: 0.013357 deg
- Worst RMS estimation error: 0.018918 deg
- Worst settling time: 11.790 s
- Maximum wheel speed: 4286.160 rpm
- Speed-limit cases: 0
- Worst final gyro-bias error magnitude: 0.004634 deg/s

## Three-Axis Simulink Model

`cubesat_attitude_3axis.slx` preserves the original single-axis model and implements the asymmetric quaternion-controlled spacecraft with reaction wheels.

- Final attitude: approximately `[-0.000174, -0.000001, -0.000003] deg`
- Final control error: 0.000174 deg
- Settling time: 9.9304 s
- Maximum torque: 0.002 N m per axis
- Maximum wheel speeds: `[3345.8, 1470.1, 1124.8] rpm`
- Maximum quaternion norm error: `8.9427e-11`

These results closely match the standalone MATLAB simulation.

## C Implementation

The C implementation preserves the original single-axis controller and adds a normalized quaternion PD controller, shortest-rotation sign handling, per-axis saturation, reaction-wheel torque and speed limiting, and an integrated command-chain test.

```text
100% tests passed, 0 tests failed out of 4
```

Build and test from a Visual Studio Developer PowerShell:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Key Entry Points

```matlab
init_simulink_3axis
build_simulink_3axis
simulink_result = sim('cubesat_attitude_3axis');
run_attitude_monte_carlo
run_estimated_state_monte_carlo
```

## Project Structure

```text
CubeSat-ADCS-Simulation/
|-- attitude_control/   MATLAB ADCS, estimator, actuator, disturbance,
|                       Monte Carlo, and Simulink-generation files
|-- orbital_dynamics/   Original two-body orbit files
|-- c_implementation/   C controllers, wheel actuator, tests, and CMake
|-- results/            Saved plots and numerical results
|-- README.md
```

## Tools

- MATLAB R2025b
- Simulink
- C99
- CMake and CTest
- Microsoft Visual C/C++ Build Tools 2022

## Modelling Limitations

- Environmental models are simplified engineering models.
- Atmospheric density is constant in the current drag comparison.
- The Earth magnetic field is a centred aligned dipole approximation.
- Sensor models use Gaussian noise and gyro-bias random walk.
- Flexible-body dynamics, structural vibration, thermal effects, detailed power constraints, and communication delays are outside the current scope.
- Hardware-in-the-loop and on-orbit validation have not been performed.
