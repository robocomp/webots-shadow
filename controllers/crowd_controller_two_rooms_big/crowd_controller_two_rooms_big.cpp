#include "CrowdAgent.cpp"

#define TIME_STEP 32


int main() {

    std::unordered_map<std::string, std::vector<std::string>> miGrafo = {
        {"WAYPOINT_1", {"WAYPOINT_2"}},
        {"WAYPOINT_2", {"WAYPOINT_3"}},
        {"WAYPOINT_3", {"WAYPOINT_4"}},
        {"WAYPOINT_4", {"WAYPOINT_5"}},
        {"WAYPOINT_5", {"WAYPOINT_6"}},
        {"WAYPOINT_6", {"WAYPOINT_7"}},
        {"WAYPOINT_7", {"WAYPOINT_8"}},
        {"WAYPOINT_8", {"WAYPOINT_1"}}
    };

    CrowdAgent* agent = new CrowdAgent(miGrafo);

    while (agent->step(TIME_STEP) != -1) {
        agent->update();
    }

    return 0;
}
