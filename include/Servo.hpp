#pragma once

extern bool gpio_init_flag;

namespace Servo
{
    
    class servo
    {
        private:
            int pin;
        public:
            servo(int pin);
            void setAngle(float theta);
    };
}