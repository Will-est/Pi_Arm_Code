#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>
#include <math.h>
#include <stack>

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.01
#define clip_constant 0.1
#define max_iterations 1000
#define small_action_threshold 0.001
#define max_small_joint_action_count 7

// ------------  ARM Code --------------

arm::arm(Ligament input_ligament) 
{
    // Set the end-effector ligament
    ee_ligament = input_ligament;

    // initialize servos and their angles
    int pin = 1;
    Ligament* current = &ee_ligament; // Initialize current with the end effector
    do {
        servos.push_back(Servo::servo(pin, -M_PI/2, M_PI/2)); // creates servo corresponding to the end of the ligament
        joint_positions.push_back(0.0f); // initializes joint positions to 0
        // Move to the next ligament in the chain
        current = current->backward_attachment;
        pin++; // Increment pin for the next servo
    }while(current != nullptr);
}

void arm::moveToPosition(float target_position[3], float margin_of_error) {
    Vector3f target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3f current_position = get_current_position().cast<float>();
    Vector3f last_position = current_position;
    Vector3f distance_vector = get_distance(current_position, target_position_vec);
    
    int iteration_count = 0;
    int small_action_count = 0;
    
    // Maximum step size in radians to prevent large jumps
    const float max_step = 0.5;
    
    while (distance_vector.norm() > margin_of_error && iteration_count < max_iterations) {
        // Calculate Jacobian and its pseudo-inverse
        Matrix<float, 3, Dynamic> jacobian = calcJacobian(jacobian_step, target_position_vec);
        Matrix<float, 3, 3> jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        
        // Apply joint limit constraints
        distance_vector = constrain_joint_positions(distance_vector, current_position, jacobian, jacobian_pseudo_inverse);
        
        // Calculate joint actions
        VectorXf joint_action_vec = jacobian_pseudo_inverse * distance_vector;
        
        // Check if we're stuck in a local minimum
        if ((current_position - last_position).norm() < small_action_threshold) {
            small_action_count++;
            if (small_action_count > max_small_joint_action_count) {
                // std::cout << "Adding perturbation to escape local minimum" << std::endl;
                for (int i = 0; i < joint_action_vec.size(); i++) {
                    joint_action_vec(i) += ((((float)rand() / RAND_MAX) - 0.5f) * 10.0f);
                }
                small_action_count = 0;
            }
        } else {
            small_action_count = 0;
        }
        
        // Limit step size
        if (joint_action_vec.norm() > max_step) {
            joint_action_vec = joint_action_vec.normalized() * max_step;
        }
        
        // Update joint positions
        write_to_joints(joint_action_vec, false);
        
        // Update positions and calculate new distance vector
        last_position = current_position;
        current_position = get_current_position().cast<float>();
        distance_vector = get_distance(current_position, target_position_vec);
        
        // std::cout << "current position:\n " << current_position << std::endl;
        // std::cout << "the norm is: " << distance_vector.norm() << std::endl;
        
        iteration_count++;
    }
    
    if (iteration_count >= max_iterations) {
        std::cout << "Maximum iterations reached without convergence." << std::endl;
    }

    // Final update to servos
    write_to_joints(VectorXf::Zero(joint_positions.size()), true);
}

Vector3f arm::FK(std::vector<float> jointConfig) {
    // Start with identity transformation
    Transform<float, 3, Affine> accumulated_transform = Transform<float, 3, Affine>::Identity();
    
    // Initial position in end effector frame
    Vector3f pos(0, 0, 0);
    
    Ligament* current_ligament = &ee_ligament;
    
    // Iterate through the ligaments from end effector to base
    for (int i = 0; i < jointConfig.size() && current_ligament != nullptr; i++) {
        // Clamp joint angle to servo limits
        float joint_angle = std::max(servos[i].getMinAngle(), 
                                    std::min(jointConfig[i], 
                                            servos[i].getMaxAngle()));
        
        // Build and apply transformation matrix
        accumulated_transform = 
            AngleAxisf(joint_angle, current_ligament->rotation) * 
            Translation3f(current_ligament->translation) * 
            accumulated_transform;
        
        // Move to the next ligament in the chain
        current_ligament = current_ligament->backward_attachment;
    }

    // Apply the accumulated transformation to the initial position
    return accumulated_transform * pos;
}

Matrix<float, 3, Dynamic> arm::calcJacobian(float delta, Vector3f target_position_vec) {
    // Calculate Jacobian matrix using central difference method
    Matrix<float, 3, Dynamic> jacobian(3, joint_positions.size());
    std::vector<float> config_plus = joint_positions;
    std::vector<float> config_minus = joint_positions;
    
    for (int i = 0; i < joint_positions.size(); ++i) {
        // Store original angle
        float original_angle = joint_positions[i];
        
        // Check if joint is near limits
        bool near_upper_limit = (original_angle > servos[i].getMaxAngle() - 0.1);
        bool near_lower_limit = (original_angle < servos[i].getMinAngle() + 0.1);
        
        if (near_upper_limit || near_lower_limit) {
            // Zero out the Jacobian column if joint is near limits
            for (int j = 0; j < 3; ++j) {
                jacobian(j, i) = 0.0f;
            }
        } else {
            // Apply deltas for central difference
            config_plus[i] = original_angle + delta;
            config_minus[i] = original_angle - delta;
            
            // Calculate positions
            Vector3f pos_plus = FK(config_plus);
            Vector3f pos_minus = FK(config_minus);
            
            // Compute central difference for each dimension
            for (int j = 0; j < 3; ++j) {
                jacobian(j, i) = (pos_plus[j] - pos_minus[j]) / (2 * delta);
            }
            
            // Reset config vectors
            config_plus[i] = original_angle;
            config_minus[i] = original_angle;
        }
    }
    
    // std::cout << "Jacobian:\n" << jacobian << std::endl;
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

    // Calculate new joint positions
    std::vector<float> new_joint_positions(joint_positions.size());
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        new_joint_positions[i] = fmod(joint_positions[i] + joint_actions(i), 2 * M_PI); // Ensure angle is within [0, 2π)
    }

    // writes to servos if needed and updates positions with clipping
    for (size_t i = 0; i < servos.size(); ++i) {
        float angle = new_joint_positions[i];
        
        // Clip angle to servo limits
        if (angle < servos[i].getMinAngle()) {
            angle = servos[i].getMinAngle();
            // std::cout << "Joint " << i << " clipped to min angle: " << angle << std::endl;
        } 
        else if (angle > servos[i].getMaxAngle()) {
            angle = servos[i].getMaxAngle();
            // std::cout << "Joint " << i << " clipped to max angle: " << angle << std::endl;
        }
        
        // Update joint position with possibly clipped value
        joint_positions[i] = angle;
        
        // Write to servo if requested
        if (write_to_servos) {
            servos[i].setAngle(angle);
        }
    }
}

arm::~arm() 
{
    // Clean up any resources
    servos.clear();
    joint_positions.clear();
    
    // Print a message indicating the destructor was called
    // std::cout << "Arm destructor called, resources released." << std::endl;
}

Vector3f arm::constrain_joint_positions(Vector3f& distance_vector, const Vector3f& current_position, 
                                   const Matrix<float, 3, Dynamic>& jacobian,
                                   const Matrix<float, 3, 3>& jacobian_pseudo_inverse)
{
    // Check for joints near limits and adjust distance vector if needed
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        float lower_limit = servos[i].getMinAngle();            // Lower joint limit (0 radians)
        float upper_limit = servos[i].getMaxAngle();        // Upper joint limit (2π radians)

        // Check if joint is near limits
        bool near_lower_limit = joint_positions[i] < (lower_limit + 0.1);
        bool near_upper_limit = joint_positions[i] > (upper_limit - 0.1);
        
        if (near_lower_limit || near_upper_limit) {
            // std::cout << "Joint " << i << " near " << (near_lower_limit ? "lower" : "upper") << " limit" << std::endl;
            
            
            // Temporarily modify the joint angle
            std::vector<float> modified_config = joint_positions;
            float direction = near_lower_limit ? 1.0f : -1.0f;  // Move away from the limit
            modified_config[i] += direction * jacobian_step;
            
            // Calculate FK for modified joint positions
            Vector3f modified_pos = FK(modified_config);
            
            // Calculate the vector resulting from the joint modification
            Vector3f joint_limit_vector = (modified_pos - current_position) / jacobian_step;

            // Project this vector onto the nullspace of the Jacobian
            Matrix<float, Dynamic, Dynamic> null_space = 
                Matrix<float, Dynamic, Dynamic>::Identity(joint_positions.size(), joint_positions.size()) - 
                jacobian_pseudo_inverse * jacobian;

            // Apply the nullspace projection to your joint_limit_vector
            VectorXf joint_space_correction = null_space * joint_limit_vector;
            
            // Add the correction to the distance vector
            distance_vector += joint_space_correction;
        }
    }
    return distance_vector;
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