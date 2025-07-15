#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>
#include <math.h>

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.01
#define clip_constant 0.1
#define max_iterations 10000
#define small_action_threshold 0.001
#define max_small_joint_action_count 10



// ------------  ARM Code --------------

arm::arm(Ligament input_ligament) 
{
    // Set the end-effector ligament
    ee_ligament = input_ligament;

    // initialize servos and their angles
    int pin = 1;
    Ligament* current = &ee_ligament; // Initialize current with the end effector
    do {
        servos.push_back(Servo::servo(pin)); // creates corresponding to the end of the ligament
        joint_positions.push_back(0.0f); // initializes joint positions to 0
        // Move to the next ligament in the chain
        current = current->backward_attachment;
        pin++; // Increment pin for the next servo
    }while(current != nullptr);
}

// Function to move the arm to a specified position
void arm::moveToPosition(float target_position[3], float margin_of_error) {

    Vector3f target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3f distance_vector = get_distance(get_current_position().cast<float>(), target_position_vec);
    int iteration_count = 0;
    int small_action_count = 0;
    
    do
    {
        // gets pseudo-inverse for the joint action
        Matrix<float, 3, Dynamic> jacobian = calcJacobian(jacobian_step); // Assuming delta is 1 for Jacobian calculation
        Matrix<float, 3, 3> jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        
        // Calculate joint action using pseudo-inverse of Jacobian
        VectorXf joint_action_vec = jacobian_pseudo_inverse * distance_vector;

        // Check if the joint action is very small
        if (joint_action_vec.norm() < small_action_threshold) {
            small_action_count++;
            if (small_action_count > max_small_joint_action_count) {
                // Add a small random perturbation to escape local minimum
                std::cout << "Adding perturbation to escape local minimum" << std::endl;
                for (int i = 0; i < joint_action_vec.size(); i++) {
                    // Add random value between -0.05 and 0.05
                    joint_action_vec(i) += ((float)rand() / RAND_MAX - 0.5f) * 0.1f;
                }
                small_action_count = 0; // Reset counter after perturbation
            }
        } else {
            small_action_count = 0; // Reset counter if action is significant
        }

        // Limit the step size to prevent huge jumps
        float max_step = 0.1; // Maximum step size in radians
        if (joint_action_vec.norm() > max_step) {
            joint_action_vec = joint_action_vec.normalized() * max_step;
        }

        //writes to servos
        write_to_joints(joint_action_vec, false);

        // updates delta
        distance_vector = get_distance(get_current_position().cast<float>(), target_position_vec);

        std::cout << "current position:\n " << get_current_position() << std::endl;
        std::cout << "the norm is: " << distance_vector.norm() << std::endl;
        
        // Increment iteration count and check for max iterations
        iteration_count++;
        if (iteration_count >= max_iterations) {
            std::cout << "Maximum iterations reached without convergence." << std::endl;
            break;
        }
    }
    while(distance_vector.norm() > margin_of_error);
}                                   

Vector3f arm::FK(std::vector<float> jointConfig) {
    // Function to perform forward kinematics
    Vector3f pos(0, 0, 0); // Start position in end effector frame
    Ligament* current_ligament = &ee_ligament;
    int i = 0;

    // Start with identity transformation
    Transform<float, 3, Affine> accumulated_transform = Transform<float, 3, Affine>::Identity();

    // Iterate through the ligaments from end effector to base
    while ( (current_ligament != nullptr) && (i < joint_positions.size()) ) {
        // Create rotation transformation using joint angle
        float joint_angle = jointConfig[i];
        
        // Get rotation axis and translation from the ligament
        Vector3f rotation_axis = current_ligament->rotation;
        Vector3f translation = current_ligament->translation;
        
        // Build transformation matrix
        Transform<float, 3, Affine> transform = 
            AngleAxisf(joint_angle, rotation_axis) * 
            Translation3f(translation);
        
        // Apply this transformation to our accumulated transform
        accumulated_transform = transform * accumulated_transform;
        
        // Move to the next ligament in the chain
        current_ligament = current_ligament->backward_attachment;
        i++;
    }

    // Apply the accumulated transformation to the initial position
    return accumulated_transform * pos;
}

Matrix<float, 3, Dynamic> arm::calcJacobian(float delta) {
    // Function to calculate Jacobians using central difference method

    Matrix<float, 3, Dynamic> jacobian(3, joint_positions.size()); // 3xN Jacobian matrix where N is the number of joints
    std::vector<float> jointConfig_plus = joint_positions;
    std::vector<float> jointConfig_minus = joint_positions;
    
    for (int i = 0; i < joint_positions.size(); ++i) { // Loop over each joint
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
    return jacobian;
}

Vector3f arm::get_current_position() {
    
    return FK(joint_positions);
}

Vector3f arm::get_distance(Vector3f current_pos, Vector3f goal_pos)
{
    Vector3f result = goal_pos - current_pos;
    return result;
}

void arm::write_to_joints(VectorXf joint_actions, bool write_to_servos) 
{
    // Ensure joint_actions vector is the right size
    if (joint_actions.size() != joint_positions.size()) {
        std::cerr << "Error: joint_actions size doesn't match joint_positions size" << std::endl;
        return;
    }

    // writes to servos if needed
    if (write_to_servos) {
        for (size_t i = 0; i < servos.size(); ++i) {
            servos[i].setAngle(joint_positions[i] + joint_actions(i));
        }
    }

    // updates current joint configuration
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        joint_positions[i] = fmod(joint_positions[i] + joint_actions(i), 2 * M_PI); // Ensure angle is within [0, 2π)
    }
}

arm::~arm() 
{
    // Clean up any resources
    servos.clear();
    joint_positions.clear();


    
    // Print a message indicating the destructor was called
    std::cout << "Arm destructor called, resources released." << std::endl;
}


// ------------  Ligament Code --------------
Ligament::Ligament()
{
    this->rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    this->translation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    this->forward_attachment = nullptr;
    this->backward_attachment = nullptr;
}

Ligament::Ligament(float rotation[3], float translation[3])
{
    this->rotation = Eigen::Vector3f(rotation[0], rotation[1], rotation[2]);
    this->translation = Eigen::Vector3f(translation[0], translation[1], translation[2]);
    this->forward_attachment = nullptr;
    this->backward_attachment = nullptr;
}
Ligament::Ligament(float rotation[3], float translation[3], Ligament* forward_attachment, Ligament* backward_attachment)
{
    this->rotation = Eigen::Vector3f(rotation[0], rotation[1], rotation[2]);
    this->translation = Eigen::Vector3f(translation[0], translation[1], translation[2]);
    this->forward_attachment = forward_attachment;
    this->backward_attachment = backward_attachment;
    
    // Connect forward attachment's backward pointer to this ligament
    if (this->forward_attachment != nullptr) {
        this->forward_attachment->backward_attachment = this;
    }
    
    // Connect backward attachment's forward pointer to this ligament
    if (this->backward_attachment != nullptr) {
        this->backward_attachment->forward_attachment = this;
    }
}
Ligament::~Ligament()
{
    // Clean up resources if needed
}