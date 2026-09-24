%% Initialise Three-Axis Simulink Attitude Model

init_attitude_3axis_asymmetric;
init_reaction_wheels;

%% Inertia-scaled quaternion-controller gains
reference_inertia = 0.00666666666666667;
reference_Kp = 0.0067;
reference_Kd = 0.0107;

principal_inertia = diag(I);

Kp_3axis = reference_Kp ...
    * principal_inertia / reference_inertia;

Kd_3axis = reference_Kd ...
    * principal_inertia / reference_inertia;

%% Initial and commanded attitudes
initial_attitude_3axis = ...
    deg2rad([30; -20; 15]);

commanded_attitude_3axis = ...
    deg2rad([0; 0; 0]);

initial_quaternion_3axis = ...
    euler_to_quaternion(initial_attitude_3axis);

command_quaternion_3axis = ...
    euler_to_quaternion(commanded_attitude_3axis);

%% Complete initial state:
% [quaternion; body rate; wheel speed]
initial_state_3axis = [
    initial_quaternion_3axis;
    0;
    0;
    0;
    0;
    0;
    0
];

simulation_duration_3axis = 20;