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
        {"WAYPOINT_8", {"WAYPOINT_9"}},
        {"WAYPOINT_9", {"WAYPOINT_10"}},
        {"WAYPOINT_10", {"WAYPOINT_11"}},
        {"WAYPOINT_11", {"WAYPOINT_12"}},
        {"WAYPOINT_12", {"WAYPOINT_13"}},
        {"WAYPOINT_13", {"WAYPOINT_14"}},
        {"WAYPOINT_15", {"WAYPOINT_1"}},
        {"WAYPOINT_14", {"WAYPOINT_15"}}
    };

    CrowdAgent* agent = new CrowdAgent(miGrafo);

    while (agent->step(TIME_STEP) != -1) {
        agent->update();
    }

    return 0;
}
