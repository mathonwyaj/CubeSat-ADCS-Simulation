cmake_minimum_required(VERSION 3.10)

project(CubeSatAttitudeController C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Existing single-axis controller test
add_executable(test_controller
    test_controller.c
    attitude_controller.c
)

# New three-axis quaternion controller test
add_executable(test_quaternion_controller
    test_quaternion_controller.c
    quaternion_controller.c
)

# Reaction-wheel actuator test
add_executable(test_reaction_wheel_actuator
    test_reaction_wheel_actuator.c
    reaction_wheel_actuator.c
)

# Integrated quaternion-controller and wheel-actuator test
add_executable(test_adcs_command_chain
    test_adcs_command_chain.c
    quaternion_controller.c
    reaction_wheel_actuator.c
)

# GCC and Clang require the mathematics library.
# Microsoft Visual C provides these functions directly.
if(NOT MSVC)
    target_link_libraries(test_controller m)
    target_link_libraries(test_quaternion_controller m)
    target_link_libraries(test_reaction_wheel_actuator m)
    target_link_libraries(test_adcs_command_chain m)
endif()
enable_testing()

add_test(
    NAME single_axis_controller_test
    COMMAND test_controller
)

add_test(
    NAME quaternion_controller_test
    COMMAND test_quaternion_controller
)

add_test(
    NAME reaction_wheel_actuator_test
    COMMAND test_reaction_wheel_actuator
)

add_test(
    NAME adcs_command_chain_test
    COMMAND test_adcs_command_chain
)
