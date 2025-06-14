#include "Servo.hpp"
#include <pigpio.h>

using namespace Servo;

// constructor for servo
servo::servo(int pin, int id) : pin(pin), id(id), angle(0) {}

void servo::setAngle(int theta)
{
    // convert radians to Pulse width
    int pulse = theta * ( 2000 / 3.14159265359);
    gpioServo(this->pin, pulse);
    this->angle += theta;
}

int servo::getAngle(){return this->angle;}