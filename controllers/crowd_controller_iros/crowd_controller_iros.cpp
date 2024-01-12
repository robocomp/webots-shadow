#include "CrowdAgent.cpp"
#include <webots/Keyboard.hpp>

#define TIME_STEP 32

std::unordered_map<std::string, std::vector<std::string>> miGrafo = {
    {"WAYPOINT_1", {"WAYPOINT_2"}}
};

int previousKey = 0;
webots::Keyboard* keyboard;

int main(int argc, char **argv) {

  CrowdAgent* agent = new CrowdAgent(miGrafo);
  keyboard = agent->getKeyboard();
  keyboard->enable(TIME_STEP);

  while (agent->step(TIME_STEP) != -1) {
    agent->update();

    int key = keyboard->getKey();
    if ((key >= 0) && key != previousKey) {
        switch (key) {
            case 'R':
                agent->reset();

                // No es lo mejor, pero funcionará para mover el Shadow
                const double shadowInitialPosition[] = {-4.23973, 1.02165e-09, 0.0300533};
                agent->getFromDef("shadow")->getField("translation")->setSFVec3f(shadowInitialPosition);

                break;
        }
    }
    previousKey = key;
  }

  delete agent;
  return 0;
}
