#include "Servo.hpp"
#include <iostream>


int main()
{
     std::cout << "hello world" << std::endl;
    Servo::servo myservo(1, 9);
    myservo.setAngle(45.0);
    while(1)
    {
    }
    return 0;
}