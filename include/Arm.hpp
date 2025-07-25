#pragma once

#include "Servo.hpp"
#include <vector>
#include <Eigen/Dense>

namespace Arm 
{
    class Ligament {
        public:
            Ligament();
            Ligament(float rotation[3], float translation[3]);
            Ligament(float rotation[3], float translation[3], Ligament* forward_attachment, Ligament* backward_attachment);
            ~Ligament();

            Eigen::Vector3f rotation; // Rotation as axis-angle representation
            Eigen::Vector3f translation; // Translation vector
            // Pointers to connected ligaments
            Ligament* forward_attachment;
            Ligament* backward_attachment;
        
        private:
    
    };
    
    class arm 
    {
        public:
            void moveToPosition(float target_position[3], float margin_of_error);                              
            arm(Ligament input_ligament, float origin_offset[3]);
            ~arm();

        private:
            Eigen::Vector3f FK(std::vector<float> jointConfig);
            Eigen::Matrix<float, 3, Eigen::Dynamic> calcJacobian(float delta, Eigen::Vector3f target_position_vec);
            Eigen::Vector3f get_current_position();
            Eigen::Vector3f get_distance(Eigen::Vector3f current_pos, Eigen::Vector3f goal_pos);  
            void write_to_joints(Eigen::VectorXf joint_actions, bool write_to_servos);    
            Eigen::Vector3f constrain_joint_positions(Eigen::Vector3f& distance_vector, const Eigen::Vector3f& current_position, 
                                   const Eigen::Matrix<float, 3, Eigen::Dynamic>& jacobian,
                                   const Eigen::Matrix<float, 3, 3>& jacobian_pseudo_inverse);                                
            std::vector<float> joint_positions;
            std::vector<Servo::servo> servos;
            Ligament ee_ligament;
            Eigen::Vector3f Origin_offset;

        
    };

}