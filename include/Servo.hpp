#pragma once

extern bool gpio_init_flag;

namespace Servo
{
    
    class servo
    {
        private:
            int id;
            int pin;
            float angle;

        public:
            servo(int id, int pin);
            void setAngle(float theta);
            float getAngle();
    };
}