#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>
#include <random>

#define DEBUG 0

#ifndef CROWDAGENT_H
#define CROWDAGENT_H

class CrowdAgent : public webots::Supervisor {
public:

    CrowdAgent(const std::unordered_map<std::string, std::vector<std::string>>& graph);
    void update();
    webots::Node* agentNode;

private:
    std::string currentDestination;
    double movementSpeed = 0.7f;

    // Estructura para guardar el grafo.
    std::unordered_map<std::string, std::vector<std::string>> graph;
    // Lista de waypoints
    std::map<std::string, webots::Node*> waypoints;

    std::string getNextDestination(const std::string& currentDestination);
    void addConnection(const std::string& from, const std::string& to);
    bool hasArrivedToDestination();
    void initializeWaypoints();
    void initializeGraphConnections();
    std::string Vector3toString(const double* vec);
    std::string getClosestWaypoint(const double* position);
    void printWaypointConnections(const std::string& waypointName);
    void printAllConnections();
    void checkArrival();
    void moveToDestination();
};

#endif // CROWDAGENT_H