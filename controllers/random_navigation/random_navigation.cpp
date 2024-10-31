#include <webots/Robot.hpp>
#include "RandomNavigation.cpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  RandomNavigation robot(2.0f);
  robot.setSpeedMovement(0.3f);

  int timeStep = 32;

  while (robot.step(timeStep) != -1) {
    robot.update();
  };

  // Enter here exit cleanup code.
  return 0;
}
