#pragma once

#define PI 3.14159265358979323846

extern bool gpio_init_flag;

namespace Servo
{
    
    class servo
    {
        private:
            int pin;
            float upper_joint_limit;
            float lower_joint_limit;
        public:
            servo(int pin);
            servo(int pin, float upper_joint_limit, float lower_joint_limit);
            float getMinAngle();
            float getMaxAngle();
            void setAngle(float theta);
    };
}