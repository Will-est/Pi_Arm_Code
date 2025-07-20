#include "Servo.hpp"
// #include <pigpio.h>
#include <iostream>

bool gpio_init_flag = false;

using namespace Servo;

// constructor for servo
servo::servo(int pin) : pin(pin)
{
    // if(!gpio_init_flag)
    // {
    //     // Initialize GPIO pin for servo control
    //     if (gpioInitialise() < 0) {
    //         // Handle initialization error
    //         std::cerr << "The init code didn't work" << std::endl;
    //         throw std::runtime_error("Failed to initialize GPIO");
    //         return;
    //     }
    //     else
    //     {
    //         std::cout << "The init code worked" << std::endl;
    //     }

    //     gpio_init_flag = true; // sets flag to prevent gpioInitialize being called later

    // }
    // gpioSetMode(pin, PI_OUTPUT);
}
// constructor for servo with min/max angle limits
servo::servo(int pin, float min_angle, float max_angle) : pin(pin), lower_joint_limit(min_angle), upper_joint_limit(max_angle)
{
    // if(!gpio_init_flag)
    // {
    //     // Initialize GPIO pin for servo control
    //     if (gpioInitialise() < 0) {
    //         // Handle initialization error
    //         std::cerr << "The init code didn't work" << std::endl;
    //         throw std::runtime_error("Failed to initialize GPIO");
    //         return;
    //     }
    //     else
    //     {
    //         std::cout << "The init code worked" << std::endl;
    //     }

    //     gpio_init_flag = true; // sets flag to prevent gpioInitialize being called later

    // }
}


void servo::setAngle(float theta) 
{
    // // convert radians to Pulse width
    int pulse = (int) (1500 + ((theta/PI)*2000));

    // clips angles to 0 - 180 (consider adding print statment here to say if we clipped)
    if (pulse < 0) {
        pulse = 0;
    } else if (pulse > 2500) {
        pulse = 2500;
    }
    // gpioServo(this->pin, pulse);
}

float servo::getMinAngle() {
    return lower_joint_limit;
}

float servo::getMaxAngle() {
    return upper_joint_limit;
}