#include "Servo.hpp"
#include <pigpio.h>
#include <iostream>

bool gpio_init_flag = false;

using namespace Servo;

// constructor for servo
servo::servo(int pin, int id) : pin(pin), id(id), angle(0) 
{
    if(!gpio_init_flag)
    {
        // Initialize GPIO pin for servo control
        if (gpioInitialise() < 0) {
            // Handle initialization error
            std::cerr << "The init code didn't work" << std::endl;
            throw std::runtime_error("Failed to initialize GPIO");
            return;
        }
        else
        {
            std::cout << "The init code did work" << std::endl;
        }

        gpio_init_flag = true; // sets flag to prevent gpioInitialize being called later

    }
    gpioSetMode(pin, PI_OUTPUT);
}

void servo::setAngle(float theta)
{
    // convert radians to Pulse width
    int pulse = (int) (theta * ( 2000 / 3.14159265359));
    // gpioServo(this->pin, pulse);
    gpioServo(this->pin, 1000);
    this->angle += theta;
}

float servo::getAngle(){return this->angle;}