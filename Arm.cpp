#include "Servo.hpp"
#include "Arm.hpp"

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.01

arm::arm()
{
    //initialize servos and stuff
}

// Function to move the arm to a specified position
void arm::moveToPosition(int target_position[3], int margin_of_error) {

    int joint_action[3]; // joint action that will be used to write to servos
    Vector3d target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3d delta_vector = target_position_vec - get_current_position();
    int delta = delta_vector.norm();

    do
    {
        // gets pseudo-inverse for the joint action
        MatrixXd jacobian = calcJacobian(jacobian_step); // Assuming delta is 1 for Jacobian calculation
        MatrixXd jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        VectorXd joint_action_vec = jacobian_pseudo_inverse * delta_vector;

        // translates to joint action vector
        for (int i = 0; i < 3; ++i) {
            joint_action[i] = static_cast<int>(joint_action_vec[i]);
        }

        //writes to servos
        base_servo.setAngle(joint_action[0]);
        rotator_servo.setAngle(joint_action[1]);
        elbow_servo.setAngle(joint_action[2]);

        // updates delta
        delta_vector = target_position_vec - get_current_position();
        delta = delta_vector.norm();

    }
    while(delta > margin_of_error);
}

VectorXd arm::FK(int jointConfig[3]) {
    // Function to perform forward kinematics
    // Implementation goes here
    return transform_to_base_frame( jointConfig[0],
         transform_to_rotator_frame( jointConfig[1],
            transform_to_elbow_frame( jointConfig[2] ) 
        ) 
    ); 
}

VectorXd arm::transform_to_elbow_frame(int elbow_rotation_angle) {
    // Function to transform coordinates to the elbow frame
    Vector3f pos(0, 0, 0);
    current_elbow_rotation_angle = base_rotation_angle * M_PI / 180.0 + current_base_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = Translation3f(forearm_length, 0, 0) * AngleAxisf(current_base_rotation_angle, Vector3f::UnitY());
    return transform * pos;
}

VectorXd arm::transform_to_rotator_frame(int rotator_rotation_angle, VectorXd pos) {
    // Function to transform coordinates to the rotator frame
    current_rotator_rotation_angle = base_rotation_angle * M_PI / 180.0 + current_base_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = Translation3f(base_arm_length, 0, 0) * AngleAxisf(current_base_rotation_angle, Vector3f::UnitY());
    return transform * pos;
}

VectorXd arm::transform_to_base_frame(int base_rotation_angle, VectorXd pos) {
    // Function to transform coordinates to the base frame
    current_base_rotation_angle = base_rotation_angle * M_PI / 180.0 + current_base_rotation_angle; // Convert angle to radians and updates
    Transform<float,3,Affine> transform = Translation3f(0, 0, base_height) * AngleAxisf(current_base_rotation_angle, Vector3f::UnitY());
    return transform * pos;
}

MatrixXd arm::calcJacobian(int delta) {
    // Function to calculate Jacobians
    MatrixXd jacobian(3, 3); // 3x3 Jacobian matrix for 3 joints and 3 dimensions (x, y, z)
    VectorXd current_position = get_current_position();
    int jointConfig[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    for (int i = 0; i < 3; ++i) { // Loop over each joint
        int original_angle = jointConfig[i];
        jointConfig[i] += delta; // Apply small change to the joint angle
        VectorXd offset_position = FK(jointConfig);
        jointConfig[i] = original_angle; // Reset the joint angle

        for (int j = 0; j < 3; ++j) { // Loop over each dimension (x, y, z)
            jacobian(j, i) = (offset_position[j] - current_position[j]) / delta;
        }
    }
    return jacobian; // Placeholder return value
}

VectorXd arm::get_current_position() {
    int jointConfig[3] = {current_base_rotation_angle, current_rotator_rotation_angle, current_elbow_rotation_angle};
    return FK(jointConfig);
}

