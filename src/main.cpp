#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>


int main()
{
    
    // Initialize the arm with example lengths
    Arm::arm myArm(1.0f, 1.0f, 1.0f);     

    float goal_pose[] = {0.5f, 0.5f, 0.5f};
    myArm.moveToPosition(goal_pose,0.001);
    

    return 0;
}