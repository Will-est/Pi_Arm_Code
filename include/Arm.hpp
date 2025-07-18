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

            // Rotation as axis-angle representation
            Eigen::Vector3f rotation;
            
            // Translation vector
            Eigen::Vector3f translation;
            
            // Pointers to connected ligaments
            Ligament* forward_attachment;
            Ligament* backward_attachment;
        
        private:
            // Add any private members if needed
    };
    
    class arm 
    {
        public:
            void moveToPosition(float target_position[3], float margin_of_error);
            Eigen::Vector3f FK(std::vector<float> jointConfig);
            Eigen::Matrix<float, 3, Eigen::Dynamic> calcJacobian(float delta, Eigen::Vector3f target_position_vec);
            Eigen::Vector3f get_current_position();
            Eigen::Vector3f get_distance(Eigen::Vector3f current_pos, Eigen::Vector3f goal_pos);  
            void write_to_joints(Eigen::VectorXf joint_actions, bool write_to_servos);                                    
            arm(Ligament input_ligament);
            ~arm();
            std::vector<float> joint_positions;


        private:
            Ligament ee_ligament;
            std::vector<Servo::servo> servos;
    };

}