#include "Servo.hpp"


namespace Arm 
{
    class arm 
    {
        public:
            void moveToPosition(int target_position[3], int margin_of_error);
            VectorXd FK(int jointConfig[]);
            VectorXd transform_to_elbow_frame(int elbow_rotation_angle );
            VectorXd transform_to_rotator_frame(int rotator_rotation_angle, VectorXd pos);
            VectorXd transform_to_base_frame(int base_rotation_angle, VectorXd pos);
            MatrixXd arm::calcJacobian(int delta);
            VectorXd arm::get_current_position();
            arm();

        private:
            int current_elbow_rotation_angle;
            int current_rotator_rotation_angle;
            int current_base_rotation_angle;    
            int forearm_length;
            int base_arm_length;
            int base_height;
            Servo::servo base_servo;
            Servo::servo rotator_servo;
            Servo::servo elbow_servo;
    };
}