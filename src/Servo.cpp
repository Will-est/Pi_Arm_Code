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
    int pulse = (int) (theta * ( 2500 / 180));

    // clips angles to 0 - 180 (consider adding print statment here to say if we clipped)
    if (pulse < 0) {
        pulse = 0;
    } else if (pulse > 2500) {
        pulse = 2500;
    }
    // gpioServo(this->pin, pulse);
    gpioServo(this->pin, pulse); // Adjusting pulse width to start from 500us
    this->angle += theta;
}

float servo::getAngle(){return this->angle;}