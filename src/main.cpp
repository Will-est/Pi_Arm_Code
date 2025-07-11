#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>



int main()
{
    
    // Initialize the arm with example lengths
    Arm::arm myArm(1.0f, 1.0f, 1.0f);     

    float target_position[] = {0.5f, 0.5f, 1.0f};
    myArm.moveToPosition(target_position, 0.1f);
    std::cout << "my current position is "<< myArm.get_current_position() << std::endl;

    return 0;
}