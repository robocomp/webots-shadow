#include "CrowdAgent.cpp"
#include <webots/Keyboard.hpp>

#define TIME_STEP 32

std::unordered_map<std::string, std::vector<std::string>> miGrafo = {
    {"WAYPOINT_1", {"WAYPOINT_2"}},
    {"WAYPOINT_2", {"WAYPOINT_3"}},
    {"WAYPOINT_3", {"WAYPOINT_4"}},

    {"WAYPOINT_5", {"WAYPOINT_6"}},
    {"WAYPOINT_6", {"WAYPOINT_7"}}
};

int previousKey = 0;
webots::Keyboard* keyboard;

int main(int argc, char **argv) {

  CrowdAgent* agent = new CrowdAgent(miGrafo);
  keyboard = agent->getKeyboard();
  keyboard->enable(TIME_STEP);

  const double* initialPosition = agent->agentNode->getPosition();

  while (agent->step(TIME_STEP) != -1) {
    agent->update();

    int key = keyboard->getKey();
    if ((key >= 0) && key != previousKey) {
        switch (key) {
            case 'R':
                agent->agentNode->getField("translation")->setSFVec3f(initialPosition);
                break;
        }
    }
    previousKey = key;
  }

  delete agent;
  return 0;
}
