#include "RandomNavigation.cpp"

#define TIME_STEP 32


int main() {


    RandomNavigation agent(2.0);

    while (agent.step(TIME_STEP) != -1) {
        agent.update();
    }
  

    return 0;
}