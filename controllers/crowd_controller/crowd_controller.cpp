#include <webots/Supervisor.hpp>
#include "CrowdManager.cpp"
#include <string>

#define TIME_STEP 32
#define DEBUG 0

class CrowdAgent : public webots::Supervisor {
public:

    CrowdAgent() {
        // Obtener el nodo del agente usando el DEF name.
        std::string defName = getName();
        agentNode = getFromDef(defName);

        // Inicializar el destino actual con su waypoint mas cercano.
        CrowdManager& manager = CrowdManager::getInstance(*this);
        currentDestination = manager.getClosestWaypoint(agentNode->getField("translation")->getSFVec3f());

        std::cout << "Human: " << defName << " Closest Waypoint: " << currentDestination << std::endl;  
    }

    void moveToDestination() {

        if (agentNode) {
            // Aquí obtendrías la posición del waypoint de destino.
            webots::Node* destinationNode = getFromDef(currentDestination);
            if (destinationNode) {
                const double* destinationPosition = destinationNode->getField("translation")->getSFVec3f();

                // Calcular la dirección hacia el destino
                double velocity[3] = {
                        destinationPosition[0] - agentNode->getField("translation")->getSFVec3f()[0],
                        destinationPosition[1] - agentNode->getField("translation")->getSFVec3f()[1],
                        0
                };

                // Normalizar el vector dirección
                double magnitude = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
                if (magnitude > 0) {
                    velocity[0] /= magnitude;
                    velocity[1] /= magnitude;
                }

                // Calcular el vector de velocidad escalando el vector dirección por la velocidad deseada
                velocity[0] = velocity[0] * movementSpeed;
                velocity[1] = velocity[1] * movementSpeed;

                // Calcular el ángulo de rotación basado en el vector dirección
                double angle = atan2(velocity[1], velocity[0]);
                // Establecer la rotación del agente. Aquí asumimos que la rotación es alrededor del eje Z.
                double rotation[4] = {0, 0, 1, angle}; // {x, y, z, angle}

                agentNode->getField("rotation")->setSFRotation(rotation);
                agentNode->setVelocity(velocity);
            }
        }
    }


    void checkArrival() {
        // Aquí revisarías si el agente ha llegado a su destino.
        // Si ha llegado, entonces pides un nuevo destino al CrowdManager.
        if (hasArrivedToDestination()) {

            CrowdManager& manager = CrowdManager::getInstance(*this);
            currentDestination = manager.getNextDestination(currentDestination);

            #if DEBUG
                std::cout << "[ " << getName() << "] " << "Destination arrived. New destination: " << currentDestination << std::endl;
            #endif
        }
    }

    bool hasArrivedToDestination() {
        // Aquí pondrías la lógica para determinar si el agente ha llegado a su destino.
        // Puedes basarte en una pequeña distancia entre el agente y el destino.
        // (De nuevo, este es pseudocódigo y podrías necesitar una lógica más compleja).

        webots::Node* destinationNode = getFromDef(currentDestination);
        if (destinationNode) {
            const double* destinationPosition = destinationNode->getField("translation")->getSFVec3f();
            const double* agentPosition = agentNode->getField("translation")->getSFVec3f();

            double distance = sqrt(
                    pow(agentPosition[0] - destinationPosition[0], 2) +
                    pow(agentPosition[1] - destinationPosition[1], 2)
            );

            return distance < 0.1;  // Suponiendo 0.1 como un umbral de llegada.
        }
        return false;
    }

private:
    webots::Node* agentNode;
    std::string currentDestination;
    double movementSpeed = 0.7f;

};


int main() {
    CrowdAgent* agent = new CrowdAgent();

    while (agent->step(TIME_STEP) != -1) {
        agent->moveToDestination();
        agent->checkArrival();
    }

    return 0;
}
