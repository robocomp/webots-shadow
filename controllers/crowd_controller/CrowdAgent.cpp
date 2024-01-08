#include "CrowdAgent.h"

CrowdAgent::CrowdAgent() {
    // Obtener el nodo del agente usando el DEF name.
    initializeWaypoints();
    initializeGraphConnections();

    std::string defName = getName();
    agentNode = getFromDef(defName);

    currentDestination = getClosestWaypoint(agentNode->getField("translation")->getSFVec3f());

    std::cout << "Human: " << defName << " Closest Waypoint: " << currentDestination << std::endl;  
}

void CrowdAgent::checkArrival() {
    // Aquí revisarías si el agente ha llegado a su destino.
    // Si ha llegado, entonces pides un nuevo destino al Crowd
    if (hasArrivedToDestination()) {

        currentDestination = getNextDestination(currentDestination);

        #if DEBUG
            std::cout << "[ " << getName() << "] " << "Destination arrived. New destination: " << currentDestination << std::endl;
        #endif
    }
}

void CrowdAgent::moveToDestination() {

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

// Retorna un nuevo destino basado en el destino actual.
std::string CrowdAgent::getNextDestination(const std::string& currentDestination) {
    auto connections = graph.find(currentDestination);

    if (connections != graph.end() && !connections->second.empty()) {


        // Genera un índice aleatorio entre 0 y connections->second.size() - 1
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, connections->second.size() - 1);
        int randomIndex = dist(gen);

        // Retorna el destino conectado en el índice aleatorio
        return connections->second[randomIndex];
    }
    return "";
}

// Añadir conexión entre waypoints.
void CrowdAgent::addConnection(const std::string& from, const std::string& to) {
    graph[from].push_back(to);
}

bool CrowdAgent::hasArrivedToDestination() {
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

void CrowdAgent::initializeWaypoints() {
    // Almacenamos nodo padre
    webots::Node* waypointsNode = this->getFromDef("WAYPOINTS");
    if (!waypointsNode){
        std::cerr << "Group parent node with DEF 'WAYPOINTS' not found." << std::endl;
        return;
    }
        

    // Accedemos a los hijos del nodo padre
    webots::Field* childrenField = waypointsNode->getField("children");
    if (!childrenField || childrenField->getType() != webots::Field::MF_NODE){
        std::cerr << "Node with DEF 'WAYPOINTS' needs to be a Group node." << std::endl;
        return;
    }

    int numberOfNodes = childrenField->getCount();
    // Recorrer todos los nodos bajo WAYPOINTS y encontrar aquellos que son waypoints.
    for (int i = 0; i < numberOfNodes; i++) {

        webots::Node* childNode = childrenField->getMFNode(i);
        if(!childNode || childNode->getTypeName() != "Pose")
            continue;

        std::string nodeName = childNode->getDef();
        // Si su DEF comienza con el nombre WAYPOINT_
        if (nodeName.find("WAYPOINT_") != std::string::npos)
            // Lo almacenamos como waypoint
            waypoints[nodeName] = childNode;
    }
}

void CrowdAgent::initializeGraphConnections() {
    // Inicializar el grafo con los waypoints y sus conexiones.

    addConnection("WAYPOINT_1", "WAYPOINT_2");
    
    addConnection("WAYPOINT_2", "WAYPOINT_1");
    addConnection("WAYPOINT_2", "WAYPOINT_3");

    addConnection("WAYPOINT_3", "WAYPOINT_2");

    addConnection("WAYPOINT_4", "WAYPOINT_5");

    addConnection("WAYPOINT_5", "WAYPOINT_4");

    addConnection("WAYPOINT_6", "WAYPOINT_7");

    addConnection("WAYPOINT_7", "WAYPOINT_6");
    addConnection("WAYPOINT_7", "WAYPOINT_8");

    addConnection("WAYPOINT_8", "WAYPOINT_9");

    addConnection("WAYPOINT_9", "WAYPOINT_8");

    addConnection("WAYPOINT_10", "WAYPOINT_11");

    addConnection("WAYPOINT_11", "WAYPOINT_10");
    addConnection("WAYPOINT_11", "WAYPOINT_12");

    addConnection("WAYPOINT_12", "WAYPOINT_11");
    addConnection("WAYPOINT_12", "WAYPOINT_13");

    addConnection("WAYPOINT_13", "WAYPOINT_12");
    
    addConnection("WAYPOINT_14", "WAYPOINT_15");
    
    addConnection("WAYPOINT_15", "WAYPOINT_14");
    
    addConnection("WAYPOINT_16", "WAYPOINT_17");
    addConnection("WAYPOINT_17", "WAYPOINT_18");
    addConnection("WAYPOINT_18", "WAYPOINT_19");
    addConnection("WAYPOINT_19", "WAYPOINT_20");
    addConnection("WAYPOINT_20", "WAYPOINT_16");
}

std::string CrowdAgent::Vector3toString(const double* vec) {
    if (vec == nullptr) {
        std::cerr << "Error: puntero nulo proporcionado." << std::endl;
        return std::string();
    }

    std::string stringVector = "[" + std::to_string(vec[0]) + ", " + std::to_string(vec[1]) + ", " + std::to_string(vec[2]) + "]";
    return stringVector;
}

// Función para obtener el waypoint más cercano a una posición dada.
std::string CrowdAgent::getClosestWaypoint(const double* position) {
    
    double minDistance = std::numeric_limits<double>::max();
    std::string closestWaypoint = "";

    for (const auto& waypoint : waypoints) {
        const double* waypointPosition = waypoint.second->getField("translation")->getSFVec3f();

        #if DEBUG
            std::cout << waypoint.first << ": " << Vector3toString(waypointPosition) << std::endl;
        #endif

        // Calcula la distancia a cada waypoint en los ejes X e Y.
        double distance = sqrt(
                pow(position[0] - waypointPosition[0], 2) +
                pow(position[1] - waypointPosition[1], 2)
        );

        if (distance < minDistance) {
            minDistance = distance;
            closestWaypoint = waypoint.first;
        }
    }

    return closestWaypoint;
}

// Método para imprimir las conexiones de un único waypoint
void CrowdAgent::printWaypointConnections(const std::string& waypointName) {
    auto it = graph.find(waypointName);
    if (it != graph.end()) {
        std::cout << waypointName << " está conectado con: ";
        for (const auto& connectedWaypoint : it->second) {
            std::cout << connectedWaypoint << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Waypoint " << waypointName << " no encontrado en el grafo." << std::endl;
    }
}

// Método para imprimir todos los waypoints y sus conexiones
void CrowdAgent::printAllConnections() {
    for (const auto& pair : graph) {
        printWaypointConnections(pair.first);
    }
}