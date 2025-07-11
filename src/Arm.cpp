#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>
#include <math.h>

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.01
#define clip_constant 0.1

arm::arm(float forearm_length, float base_arm_length, float base_height) 
    : forearm_length(forearm_length), base_arm_length(base_arm_length), base_height(base_height),
      base_servo(1, 9), // Example ID and pin
      rotator_servo(2, 10), // Example ID and pin
      elbow_servo(3, 11) // Example ID and pin
{

    // note, I am going to need to have some sort of default/configurable way to set the pins for each of the servos


    // set the angles internal to the arm to 0
    current_elbow_rotation_angle = 0.5;
    current_rotator_rotation_angle = 0;
    current_base_rotation_angle = 0;
    
    // sets the angles internal to the servos to zero
    base_servo.setAngle(0);
    rotator_servo.setAngle(0);
    elbow_servo.setAngle(0);
}

// Function to move the arm to a specified position
void arm::moveToPosition(float target_position[3], float margin_of_error) {

    float joint_action[3]; // joint action that will be used to write to servos
    Vector3f target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3f distance_vector = get_distance(get_current_position().cast<float>(), target_position_vec);

    do
    {
        // gets pseudo-inverse for the joint action
        Matrix3f jacobian = calcJacobian(jacobian_step, target_position); // Assuming delta is 1 for Jacobian calculation
        Matrix3f jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Vector3f joint_action_vec = jacobian_pseudo_inverse * distance_vector;

        // Limit the step size to prevent huge jumps
        float max_step = 0.1; // Maximum step size in radians
        if (joint_action_vec.norm() > max_step) {
            joint_action_vec = joint_action_vec.normalized() * max_step;
        }

        // translates to joint action vector
        for (int i = 0; i < 3; ++i) {
            joint_action[i] = static_cast<float>(joint_action_vec[i]);
        }

        //writes to servos
        write_to_joints(joint_action);

        // updates delta
        distance_vector = get_distance(get_current_position().cast<float>(), target_position_vec);

        std::cout << "current position:\n " << get_current_position() << std::endl;
        std::cout << "the norm is: " << distance_vector.norm() << std::endl;
    }
    while(distance_vector.norm() > margin_of_error);
}                                   

Vector3f arm::FK(float* jointConfig) {
    // Function to perform forward kinematics
    // Implementation goes here
    return transform_to_base_frame( jointConfig[0],
         transform_to_rotator_frame( jointConfig[1],
            transform_to_elbow_frame( jointConfig[2] ) 
        ) 
    ); 
}

Vector3f arm::transform_to_elbow_frame(float elbow_rotation_angle) {
    // Function to transform coordinates to the elbow frame
    Vector3f pos(0, 0, 0);
    float current_joint_anlge = elbow_rotation_angle + current_elbow_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = AngleAxisf(current_joint_anlge, Vector3f::UnitY()) * Translation3f(forearm_length, 0, 0);
    return (transform * pos);
}

Vector3f arm::transform_to_rotator_frame(float rotator_rotation_angle, Vector3f pos) {
    // Function to transform coordinates to the rotator frame
    float current_joint_anlge = rotator_rotation_angle + current_rotator_rotation_angle; 
    Transform<float,3,Affine> transform = AngleAxisf(current_joint_anlge, Vector3f::UnitY()) * Translation3f(base_arm_length, 0, 0);
    return transform * pos;
}

Vector3f arm::transform_to_base_frame(float base_rotation_angle, Vector3f pos) {
    // Function to transform coordinates to the base frame
    float current_joint_anlge = base_rotation_angle + current_base_rotation_angle;
    Transform<float,3,Affine> transform = AngleAxisf(current_joint_anlge, Vector3f::UnitZ()) * Translation3f(0, 0, base_height) ;
    return transform * pos;
}

Matrix3f arm::calcJacobian(float delta, float* target_position) {
    // Function to calculate Jacobians using central difference method

    Matrix3f jacobian(3, 3); // 3x3 Jacobian matrix for 3 joints and 3 dimensions (x, y, z)
    float jointConfig_plus[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    float jointConfig_minus[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    
    for (int i = 0; i < 3; ++i) { // Loop over each joint
        float original_angle = jointConfig_plus[i]; // sets the original angle

        // Apply positive and negative deltas
        jointConfig_plus[i] += delta;
        jointConfig_minus[i] -= delta;

        Vector3f offset_position_plus = FK(jointConfig_plus);
        Vector3f offset_position_minus = FK(jointConfig_minus);
        
        // Reset joint angles
        jointConfig_plus[i] = original_angle;
        jointConfig_minus[i] = original_angle;

        for (int j = 0; j < 3; ++j) { // Loop over each dimension (x, y, z)
            // Central difference formula for numerical derivative
            jacobian(j, i) = (offset_position_plus[j] - offset_position_minus[j]) / (2 * delta);
        }
    }
    std::cout << "The jacobian is:\n " << jacobian << std::endl;
    return jacobian; // Placeholder return value
}

Vector3f arm::get_current_position() {
    float jointConfig[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    return FK(jointConfig);
}

Vector3f arm::get_distance(Vector3f current_pos, Vector3f goal_pos)
{
    Vector3f result = goal_pos - current_pos;
    return result;
}

void arm::write_to_joints(float* joint_actions)
{
    // writes to servos
    base_servo.setAngle(joint_actions[0]);
    rotator_servo.setAngle(joint_actions[1]);
    elbow_servo.setAngle(joint_actions[2]);

    // updates current joint configuration
    current_base_rotation_angle = fmod(current_base_rotation_angle + joint_actions[0], 2 * M_PI); // Ensure angle is within [0, 2π)
    current_rotator_rotation_angle = fmod(current_rotator_rotation_angle + joint_actions[1], 2 * M_PI);
    current_elbow_rotation_angle = fmod(current_elbow_rotation_angle + joint_actions[2], 2 * M_PI);
}