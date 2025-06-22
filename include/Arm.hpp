#include "Servo.hpp"
#include <Eigen/Dense>

#pragma once

namespace Arm 
{
    class arm 
    {
        public:
            void moveToPosition(float target_position[3], float margin_of_error);
            Eigen::Vector3f FK(float jofloatConfig[]);
            Eigen::Vector3f transform_to_elbow_frame(float elbow_rotation_angle );
            Eigen::Vector3f transform_to_rotator_frame(float rotator_rotation_angle, Eigen::Vector3f pos);
            Eigen::Vector3f transform_to_base_frame(float base_rotation_angle, Eigen::Vector3f pos);
            Eigen::Matrix3f calcJacobian(float delta);
            Eigen::Vector3f get_current_position();
            arm(float forearm_length, float base_arm_length, float base_height);

        private:
            float current_elbow_rotation_angle;
            float current_rotator_rotation_angle;
            float current_base_rotation_angle;    
            float forearm_length;
            float base_arm_length;
            float base_height;
            Servo::servo base_servo;
            Servo::servo rotator_servo;
            Servo::servo elbow_servo;
    };
}