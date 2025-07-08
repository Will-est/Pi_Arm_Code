#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.0001

arm::arm(float forearm_length, float base_arm_length, float base_height) 
    : forearm_length(forearm_length), base_arm_length(base_arm_length), base_height(base_height),
      base_servo(1, 9), // Example ID and pin
      rotator_servo(2, 10), // Example ID and pin
      elbow_servo(3, 11) // Example ID and pin
{

    // note, I am going to need to have some sort of default/configurable way to set the pins for each of the servos


    // set the angles internal to the arm to 0
    current_elbow_rotation_angle = 0;
    current_rotator_rotation_angle = 0;
    current_base_rotation_angle = 0;
    
    // sets the angles internal to the servos to zero
    base_servo.setAngle(0);
    rotator_servo.setAngle(0);
    elbow_servo.setAngle(0);
}

// Function to move the arm to a specified position
void arm::moveToPosition(float target_position[3], float margin_of_error) {

    // float joint_action[3]; // joint action that will be used to write to servos
    // Vector3d target_position_vec(target_position[0], target_position[1], target_position[2]);
    // Vector3d delta_vector = target_position_vec - get_current_position();
    // float delta = delta_vector.norm();

    float joint_action[3]; // joint action that will be used to write to servos
    Vector3f target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3f delta_vector = target_position_vec - get_current_position().cast<float>();
    float delta = delta_vector.norm();

    do
    {
        // gets pseudo-inverse for the joint action
        Matrix3f jacobian = calcJacobian(jacobian_step); // Assuming delta is 1 for Jacobian calculation
        Matrix3f jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Vector3f joint_action_vec = jacobian_pseudo_inverse * delta_vector;

        // translates to joint action vector
        for (int i = 0; i < 3; ++i) {
            joint_action[i] = static_cast<float>(joint_action_vec[i]);
        }

        //writes to servos
        base_servo.setAngle(joint_action[0]);
        rotator_servo.setAngle(joint_action[1]);
        elbow_servo.setAngle(joint_action[2]);

        // updates delta
        delta_vector = target_position_vec - get_current_position().cast<float>();
        delta = delta_vector.norm();

        std::cout << "current position:\n " << get_current_position() << std::endl;
    }
    while(delta > margin_of_error);
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
    current_elbow_rotation_angle = (elbow_rotation_angle * M_PI / 180.0) + current_elbow_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = AngleAxisf(current_elbow_rotation_angle, Vector3f::UnitY()) * Translation3f(forearm_length, 0, 0);
    return (transform * pos);
}

Vector3f arm::transform_to_rotator_frame(float rotator_rotation_angle, Vector3f pos) {
    // Function to transform coordinates to the rotator frame
    current_rotator_rotation_angle = (rotator_rotation_angle * M_PI / 180.0) + current_rotator_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = AngleAxisf(current_rotator_rotation_angle, Vector3f::UnitY()) * Translation3f(base_arm_length, 0, 0);
    return transform * pos;
}

Vector3f arm::transform_to_base_frame(float base_rotation_angle, Vector3f pos) {
    // Function to transform coordinates to the base frame
    current_base_rotation_angle = (base_rotation_angle * M_PI / 180.0) + current_base_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = AngleAxisf(current_base_rotation_angle, Vector3f::UnitY()) * Translation3f(0, 0, base_height) ;
    return transform * pos;
}

Matrix3f arm::calcJacobian(float delta) {
    // Function to calculate Jacobians
    Matrix3f jacobian(3, 3); // 3x3 Jacobian matrix for 3 joints and 3 dimensions (x, y, z)
    Vector3f current_position = get_current_position();
    float jointConfig[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    for (int i = 0; i < 3; ++i) { // Loop over each joint
        float original_angle = jointConfig[i];
        jointConfig[i] += delta; // Apply small change to the joint angle
        Vector3f offset_position = FK(jointConfig);
        jointConfig[i] = original_angle; // Reset the joint angle

        for (int j = 0; j < 3; ++j) { // Loop over each dimension (x, y, z)
            jacobian(j, i) = (offset_position[j] - current_position[j]) / delta;
        }
    }
    return jacobian; // Placeholder return value
}

Vector3f arm::get_current_position() {
    float jointConfig[3] = {0, 0, 0};
    return FK(jointConfig);
}

