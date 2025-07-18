#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>
#include <math.h>
#include <stack>

using namespace Arm;
using namespace Eigen;

#define jacobian_step 0.01
#define clip_constant 0.1
#define max_iterations 10000
#define small_action_threshold 0.001
#define max_small_joint_action_count 50



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

// Function to move the arm to a specified position
void arm::moveToPosition(float target_position[3], float margin_of_error) {

    Vector3f target_position_vec(target_position[0], target_position[1], target_position[2]);
    Vector3f distance_vector = get_distance(get_current_position().cast<float>(), target_position_vec);
    int iteration_count = 0;
    int small_action_count = 0;
    

    // Being descent into correct joint configs

    // Vectors that will be used to check how we progress after each timestep
    Vector3f current_position = get_current_position().cast<float>();
    Vector3f last_position = current_position; // Static to persist between iterations
    do
    {
        // gets pseudo-inverse for the joint action
        Matrix<float, 3, Dynamic> jacobian = calcJacobian(jacobian_step, target_position_vec); // Assuming delta is 1 for Jacobian calculation
        Matrix<float, 3, 3> jacobian_pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        

        // applying nullspace projection to the distance vector

        // Check for joints near limits and adjust distance vector if needed
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            float lower_limit = servos[i].getMinAngle();            // Lower joint limit (0 radians)
            float upper_limit = servos[i].getMaxAngle();        // Upper joint limit (2π radians)

            // Check if joint is near limits
            bool near_lower_limit = joint_positions[i] < (lower_limit + 0.1);
            bool near_upper_limit = joint_positions[i] > (upper_limit - 0.1);
            
            if (near_lower_limit || near_upper_limit) {
                std::cout << "Joint " << i << " near " << (near_lower_limit ? "lower" : "upper") << " limit" << std::endl;
                
                
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


        // Calculate joint action using pseudo-inverse of Jacobian
        VectorXf joint_action_vec = jacobian_pseudo_inverse * distance_vector;
        
        // Check if the actual displacement from the last step is very small, i.e we are stuck
        if ((current_position - last_position).norm() < small_action_threshold) 
        {
            small_action_count++;
            if (small_action_count > max_small_joint_action_count) 
            {
            // Add a small random perturbation to escape local minimum
                std::cout << "Adding perturbation to escape local minimum" << std::endl;
                for (int i = 0; i < joint_action_vec.size(); i++) {
                    // Add random value between -0.05 and 0.05
                    joint_action_vec(i) += ((((float)rand() / RAND_MAX) - 0.5f) * 10.0f);
                }
                
                // // Generate a random direction vector
                // Vector3f random_direction(
                //     ((float)rand() / RAND_MAX) - 0.5f,
                //     ((float)rand() / RAND_MAX) - 0.5f,
                //     ((float)rand() / RAND_MAX) - 0.5f
                // );
                // random_direction.normalize();
                
                // // Create a new waypoint closer to target than current position
                // target_position_vec = ( (target_position_vec + current_position)/2) + (random_direction * distance_vector.norm() * 0.25f);
                
                // // Push original target to stack
                // corrective_joint_trajectorie.push(target_position_vec);
                
                small_action_count = 0; // Reset counter after perturbation

            }
        } else {
            small_action_count = 0; // Reset counter if displacement is significant
        }
        
        // Limit the step size to prevent huge jumps
        float max_step = 0.5; // Maximum step size in radians
        if (joint_action_vec.norm() > max_step) {
            joint_action_vec = joint_action_vec.normalized() * max_step;
        }
        // Convert joint_action_vec to a float array
        float joint_action_array[joint_action_vec.size()];
        for (int i = 0; i < joint_action_vec.size(); i++) {
            joint_action_array[i] = joint_action_vec(i);
        }
        //writes to servos
        write_to_joints(joint_action_vec, false);

        // Update last and current position for next iteration
        last_position = current_position;
        current_position = get_current_position().cast<float>();

        // updates delta
        distance_vector = get_distance(current_position, target_position_vec);

        std::cout << "current position:\n " << get_current_position() << std::endl;
        std::cout << "the norm is: " << distance_vector.norm() << std::endl;
        
        iteration_count++; // Increment iteration count 

        // ---- Break conditions ----

        // Checks for max iterations
        iteration_count++;
        if (iteration_count >= max_iterations) {
            std::cout << "Maximum iterations reached without convergence." << std::endl;
            break;
        }
        // Checks if the distance vector is within the margin of error
        if(distance_vector.norm() < margin_of_error)
        {
            break; // No corrective trajectory available, stop movement
        }
    }
    while(true); // Continue until the distance vector is within the margin of error


    // After getting close with gradient descent, perform random sampling to fine-tune
    std::cout << "Fine-tuning position with random sampling..." << std::endl;
    float best_distance = distance_vector.norm();
    std::vector<float> best_joint_positions = joint_positions;

    // Try 100 random samples around current position
    for (int sample = 0; sample < 100; sample++) {
        std::vector<float> sampled_positions = joint_positions;
        
        // Create a random perturbation for each joint within limits
        for (size_t i = 0; i < joint_positions.size(); i++) {
            float min_angle = servos[i].getMinAngle();
            float max_angle = servos[i].getMaxAngle();
            float current = joint_positions[i];
            
            // Sample within a smaller range around current position
            float range = 0.5; // radians
            float lower_bound = std::max(min_angle, current - range);
            float upper_bound = std::min(max_angle, current + range);
            
            // Generate random position within bounds
            sampled_positions[i] = lower_bound + ((float)rand() / RAND_MAX) * (upper_bound - lower_bound);
        }
        
        // Calculate position and distance for this sample
        Vector3f sampled_position = FK(sampled_positions);
        Vector3f sampled_distance = get_distance(sampled_position, target_position_vec);
        float distance_norm = sampled_distance.norm();
        
        // Keep track of the best sample
        if (distance_norm < best_distance) {
            best_distance = distance_norm;
            best_joint_positions = sampled_positions;
            std::cout << "Found better position with distance: " << best_distance << std::endl;
        }
    }

    // Apply the best found position
    if (best_distance < distance_vector.norm()) {
        std::cout << "Using best sampled position with distance: " << best_distance << std::endl;
        // Directly update joint_positions with the best configuration
        joint_positions = best_joint_positions;
    }
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
        
        // Ensure joint angle is within servo limits
        if (joint_angle < servos[i].getMinAngle()) {
            joint_angle = servos[i].getMinAngle();
        } else if (joint_angle > servos[i].getMaxAngle()) {
            joint_angle = servos[i].getMaxAngle();
        }
        
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

Matrix<float, 3, Dynamic> arm::calcJacobian(float delta, Vector3f target_position_vec) {
    // Function to calculate Jacobians using central difference method

    Matrix<float, 3, Dynamic> jacobian(3, joint_positions.size()); // 3xN Jacobian matrix where N is the number of joints
    std::vector<float> jointConfig_plus = joint_positions;
    std::vector<float> jointConfig_minus = joint_positions;
    
    for (int i = 0; i < joint_positions.size(); ++i) { // Loop over each joint
        float original_angle = jointConfig_plus[i]; // sets the original angle

        // Apply positive and negative deltas
        jointConfig_plus[i] += delta;
        jointConfig_minus[i] -= delta;

        // Check if we're close to joint limits and adjust gradient accordingly
        bool near_upper_limit = (joint_positions[i] > servos[i].getMaxAngle() - 0.1);
        bool near_lower_limit = (joint_positions[i] < servos[i].getMinAngle() + 0.1);

        Vector3f offset_position_plus, offset_position_minus;
         // Normal case: use FK to calculate positions
        offset_position_plus = FK(jointConfig_plus);
        offset_position_minus = FK(jointConfig_minus);

        if (near_upper_limit || near_lower_limit) {

            offset_position_plus *= 0;
            offset_position_minus *= 0;

            // float distanceToLimit;
            // float factor;
            
            // if (near_upper_limit) {
            //     distanceToLimit = servos[i].getMaxAngle() - joint_positions[i];
            //     // Exponential term that grows as we get closer to the limit
            //     factor = exp(-4.0f * distanceToLimit) * delta;
                
            //     for (int j = 0; j < 3; ++j) {
            //         offset_position_plus[j] += factor;
            //         offset_position_minus[j] -= factor;
            //     }
            // } else { // near_lower_limit
            //     distanceToLimit = joint_positions[i] - servos[i].getMinAngle();
            //     // Exponential term that grows as we get closer to the limit
            //     factor = exp(-10.0f * distanceToLimit) * delta;
                
            //     for (int j = 0; j < 3; ++j) {
            //         offset_position_plus[j] -= factor;
            //         offset_position_minus[j] += factor;
            //     }
            // }
        }
        
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
            std::cout << "Joint " << i << " clipped to min angle: " << angle << std::endl;
        } 
        else if (angle > servos[i].getMaxAngle()) {
            angle = servos[i].getMaxAngle();
            std::cout << "Joint " << i << " clipped to max angle: " << angle << std::endl;
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