#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>


#ifndef RANDOMNAVIGATION_H
#define RANDOMNAVIGATION_H

using namespace webots;

class RandomNavigation: public webots::Supervisor {
public:

    RandomNavigation();
    void moveToRandomPosition();
    void pivotAroundInitialPosition();
    void update();
    Node *robotNode;

private:
    double range;
    int timeStep;
    const double *initialPosition;
};


#endif // RANDOMNAVIGATION_H
