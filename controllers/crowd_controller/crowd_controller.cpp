#include "CrowdAgent.h"

#define TIME_STEP 32


int main() {
    CrowdAgent* agent = new CrowdAgent();

    while (agent->step(TIME_STEP) != -1) {
        agent->moveToDestination();
        agent->checkArrival();
    }

    return 0;
}
