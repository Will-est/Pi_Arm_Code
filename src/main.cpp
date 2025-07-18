#include "Servo.hpp"
#include "Arm.hpp"
#include <iostream>



int main()
{
    
    // Create three ligaments with specified axes and translation vectors
    float baseAxis[3] = {0.0f, 0.0f, 1.0f};
    float baseTranslation[3] = {0.0f, 0.0f, 1.0f};
    Arm::Ligament base(baseAxis, baseTranslation);
    
    float rotatorAxis[3] = {0.0f, 1.0f, 0.0f};
    float rotatorTranslation[3] = {1.0f, 0.0f, 0.0f};
    Arm::Ligament rotator(rotatorAxis, rotatorTranslation, nullptr, &base);

    float eeAxis[3] = {0.0f, 1.0f, 0.0f};
    float eeTranslation[3] = {1.0f, 0.0f, 0.0f};
    Arm::Ligament ee(eeAxis, eeTranslation, nullptr, &rotator);

    // Create the arm with the three ligaments
    Arm::arm myArm(ee);

    float target_position[] = {1.0f, 1.0f, 1.0f};
    myArm.moveToPosition(target_position, 0.25f);
    std::cout << "my current position is " << myArm.get_current_position() << std::endl;

    // Print joint positions
    const auto& joint_positions = myArm.joint_positions;
    // Print joint positions
    std::cout << "Joint positions:" << std::endl;
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        std::cout << "Joint " << i << ": " 
                  << joint_positions[i] << std::endl;
    }
}