//
// Created by usuario on 30/10/24.
//

#include "RandomNavigation.h"

RandomNavigation::RandomNavigation(){
    robotNode = getFromDef("BOX"); // Asegúrate de que el DEF de tu supervisor sea "SUPERVISOR"
    if (!robotNode) {
        std::cerr << "No se encontró el nodo del supervisor." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::srand(std::time(nullptr)); // Semilla para generar números aleatorios
    initialPosition = robotNode->getField("translation")->getSFVec3f();
    moveToRandomPosition();
}

void RandomNavigation::moveToRandomPosition(){
    // Genera una posición aleatoria dentro del rango especificado
    double randomX = initialPosition[0] + ((std::rand() % 100) / 100.0 * range) - (range / 2);
    double randomY = initialPosition[1] + ((std::rand() % 100) / 100.0 * range) - (range / 2);
    
    double newPosition[3] = {randomX, randomY, initialPosition[2]};

    robotNode->getField("translation")->setSFVec3f(newPosition);
    std::cout << "Moviendo a posición: [" << randomX << ", " << randomY << ", " << initialPosition[2] << "]" << std::endl;
}

void RandomNavigation::pivotAroundInitialPosition() {
    const double pivotAngle = M_PI / 2; // Rotar 90 grados
    double rotation[4] = {0, 0, 1, pivotAngle}; // Rotar alrededor del eje Z
    robotNode->getField("rotation")->setSFRotation(rotation);
    std::cout << "Pivotando alrededor de la posición inicial." << std::endl;
}

void RandomNavigation::update() {
    moveToRandomPosition();
    // Llama a pivotAroundInitialPosition() en el momento adecuado
    // Por ejemplo, puedes usar un contador para determinar cuándo pivotar.
}