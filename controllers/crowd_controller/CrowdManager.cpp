#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <map>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <random>

#define MANAGER_DEBUG 0

class CrowdManager {
public:

    // Método para obtener la instancia Singleton.
    static CrowdManager& getInstance(webots::Supervisor& supervisor) {
        static CrowdManager instance(supervisor);
        return instance;
    }

    // Retorna un nuevo destino basado en el destino actual.
    std::string getNextDestination(const std::string& currentDestination) {
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

    // Función para obtener el waypoint más cercano a una posición dada.
    std::string getClosestWaypoint(const double* position) {
        
        double minDistance = std::numeric_limits<double>::max();
        std::string closestWaypoint = "";

        for (const auto& waypoint : waypoints) {
            const double* waypointPosition = waypoint.second->getField("translation")->getSFVec3f();

            #if MANAGER_DEBUG
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
    void printWaypointConnections(const std::string& waypointName) {
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
    void printAllConnections() {
        for (const auto& pair : graph) {
            printWaypointConnections(pair.first);
        }
    }

private:

    // Estructura para guardar el grafo.
    std::map<std::string, std::vector<std::string>> graph;
    // Lista de waypoints
    std::map<std::string, webots::Node*> waypoints;

    // Constructor privado (patrón Singleton).
    CrowdManager(webots::Supervisor& supervisor) {
        initializeWaypoints(supervisor);
        initializeGraphConnections();
    }

    // Evitar que se pueda copiar el objeto.
    CrowdManager(CrowdManager const&) = delete;
    void operator=(CrowdManager const&) = delete;

    // Añadir conexión entre waypoints.
    void addConnection(const std::string& from, const std::string& to) {
        graph[from].push_back(to);
    }

    void initializeWaypoints(webots::Supervisor& supervisor) {
        // Almacenamos nodo padre
        webots::Node* waypointsNode = supervisor.getFromDef("WAYPOINTS");
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

    void initializeGraphConnections() {
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
    }

    std::string Vector3toString(const double* vec) {
        if (vec == nullptr) {
            std::cerr << "Error: puntero nulo proporcionado." << std::endl;
            return std::string();
        }

        std::string stringVector = "[" + std::to_string(vec[0]) + ", " + std::to_string(vec[1]) + ", " + std::to_string(vec[2]) + "]";
        return stringVector;
    }
};
