#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <iostream>
#include <cstring>
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::seconds

const int TIME_STEP = 32;
const double INCREMENT = 0.4;
const double MAX_VEL = 15.0;  // You can adjust this value as required

// Declaration of methods
void robot_init();
void passive_wait(double duration);
void display_helper_message();

webots::Supervisor* robot;
webots::Node* node;

int main(int argc, char **argv) {
    webots::Keyboard keyboard;

    robot_init();
    passive_wait(2.0);

    display_helper_message();

    double velocity[6] = {0, 0, 0, 0, 0, 0};

    int pc = 0;
    keyboard.enable(TIME_STEP);

    while (robot->step(TIME_STEP) != -1) {
        int c = keyboard.getKey();
        if ((c >= 0) && c != pc) {
            switch (c) {
                case webots::Keyboard::UP:
                    std::cout << "Key UP pressed." << std::endl;
                    velocity[1] = std::min(velocity[1] + INCREMENT, MAX_VEL);
                    break;
                case webots::Keyboard::DOWN:
                    std::cout << "Key DOWN pressed." << std::endl;
                    velocity[1] = std::max(velocity[1] - INCREMENT, -MAX_VEL);
                    break;
                case webots::Keyboard::LEFT:
                    std::cout << "Key LEFT pressed." << std::endl;
                    velocity[0] = std::min(velocity[0] + INCREMENT, MAX_VEL);
                    break;
                case webots::Keyboard::RIGHT:
                    std::cout << "Key RIGHT pressed." << std::endl;
                    velocity[0] = std::max(velocity[0] - INCREMENT, -MAX_VEL);
                    break;
                case webots::Keyboard::PAGEUP:
                    std::cout << "Key PAGEUP pressed." << std::endl;
                    velocity[5] = std::max(velocity[5] - INCREMENT, -MAX_VEL);
                    break;
                case webots::Keyboard::PAGEDOWN:
                    std::cout << "Key PAGEDOWN pressed." << std::endl;
                    velocity[5] = std::min(velocity[5] + INCREMENT, MAX_VEL);
                    break;
                case webots::Keyboard::END:
                    std::cout << "Key END pressed." << std::endl;
                    for (int i = 0; i < 6; i++) {
                        velocity[i] = 0;
                    }
                    break;
            }
            node->setVelocity(velocity);
        }
        pc = c;
    }
    
    delete robot;
    return 0;
}

void robot_init() {
    robot = new webots::Supervisor();
    node = robot->getFromDef("HUMAN");
    
    std::cout << "Robot initialized." << std::endl;
}

void passive_wait(double duration) {
    // Code to make the robot wait passively for the given duration
    std::cout << "Waiting for " << duration << " seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(duration)));
    std::cout << "Wait completed." << std::endl;
}

void display_helper_message() {
    // Code to display a detailed helper message
    std::cout << "-------------------------- HUMAN INSTRUCTIONS --------------------------" << std::endl;
    std::cout << "1. Use the ARROW KEYS to control the robot's movement:" << std::endl;
    std::cout << "   - UP ARROW: Increase forward velocity" << std::endl;
    std::cout << "   - DOWN ARROW: Increase backward velocity" << std::endl;
    std::cout << "   - LEFT ARROW: Increase leftward velocity" << std::endl;
    std::cout << "   - RIGHT ARROW: Increase rightward velocity" << std::endl;
    std::cout << "2. Use PAGEUP and PAGEDOWN keys to adjust the rotation velocity component:" << std::endl;
    std::cout << "   - PAGEUP: Decrease the rotation velocity component" << std::endl;
    std::cout << "   - PAGEDOWN: Increase the rotation velocity component" << std::endl;
    std::cout << "3. Press the END key to reset all velocities to zero." << std::endl;
    std::cout << "-------------------------------------------------------------------" << std::endl;
}
