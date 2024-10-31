#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <cmath>
#include <cstdlib>
#include <array>
#include <algorithm>
#include <ctime>
#include <Eigen/Dense>

#define DEBUG 0

#ifndef RANDOMNAVIGATION_H
#define RANDOMNAVIGATION_H

using namespace webots;
using namespace Eigen;

class RandomNavigation: public webots::Supervisor {
public:

    RandomNavigation(double range);
    void update();
    Node *agentNode;
    void setSpeedMovement(float speed);

private:
    double range;
    int timeStep;
    double initialPosition[3];
    double currentDestination[3];
    float movementSpeed = 0.7f;
    Vector3d velocity;

    void checkArrival();
    void moveToDestination();
    void pivotAroundInitialPosition();
    void generateRandomDestination(double (&destination)[3]);
    bool hasArrivedToDestination();


};


#endif // RANDOMNAVIGATION_H
