#include <iostream>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>

int main() {
    try {
        // Connect to the robot
        franka::Robot robot("192.168.2.105"); 

        // Define a callback function to process the robot state
        auto print_pose =  -> bool {
            // Extract the position of the end-effector
            std::array<double, 3> position = {{
                robot_state.O_T_EE[12],
                robot_state.O_T_EE[13],
                robot_state.O_T_EE[14]
            }};

            // Extract the orientation of the end-effector
            std::array<double, 4> orientation = {{
                robot_state.O_T_EE[0], robot_state.O_T_EE[1], robot_state.O_T_EE[2], // First column
                robot_state.O_T_EE[4], robot_state.O_T_EE[5], robot_state.O_T_EE[6], // Second column
                robot_state.O_T_EE[8], robot_state.O_T_EE[9], robot_state.O_T_EE[10]  // Third column
            }};

            // Print the pose (position and orientation)
            std::cout << "End-effector position: "
                      << position[0] << ", "
                      << position[1] << ", "
                      << position[2] << std::endl;
            std::cout << "End-effector orientation: "
                      << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", "
                      << orientation[3] << ", " << orientation[4] << ", " << orientation[5] << ", "
                      << orientation[6] << ", " << orientation[7] << ", " << orientation[8] << std::endl;

            // Return false to continue looping
            return false;
        };

        // Read the robot state at 30Hz
        robot.read(print_pose, franka::Duration(1.0 / 30.0));
    } catch (franka::Exception& e) {
        // Handle exceptions
        std::cerr << e.what() << std::endl;
        return -1;
    }
    return 0;
}
