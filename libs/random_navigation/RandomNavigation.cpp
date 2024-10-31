//
// Created by usuario on 30/10/24.
//

#include "RandomNavigation.h"

RandomNavigation::RandomNavigation(double range) : range(range){
    agentNode = getSelf();
    if (!agentNode) {
        std::cerr << "No se encontró el nodo del supervisor." << std::endl;
        exit(EXIT_FAILURE);
    }
    std::srand(std::time(nullptr));
    const double* currentPosition = agentNode->getField("translation")->getSFVec3f();
    initialPosition[0] = currentPosition[0];
    initialPosition[1] = currentPosition[1];
    initialPosition[2] = currentPosition[2];
    generateRandomDestination(currentDestination);
}

void RandomNavigation::generateRandomDestination(double (&destination)[3]) {
    // Genera un ángulo y una distancia aleatorios dentro del rango
    double angle = ((std::rand() % 360) / 180.0) * M_PI;  // Ángulo aleatorio en radianes
    double distance = ((std::rand() % 100) / 100.0) * range;  // Distancia aleatoria dentro del rango
    
    // Calcula las coordenadas X e Y usando el ángulo y la distancia
    double randomX = initialPosition[0] + distance * cos(angle);
    double randomY = initialPosition[1] + distance * sin(angle);

    // Actualiza el destino
    destination[0] = randomX;
    destination[1] = randomY;
    destination[2] = initialPosition[2];  // Mantiene la misma altura en Z

    #if DEBUG
        std::cout << "DEBUG: Random Navigation: Generating random Destination -> ["
                  << randomX << ", " << randomY << ", " << initialPosition[2] << "]" << std::endl;
    #endif
}


void RandomNavigation::update() {
    moveToDestination();
    checkArrival(); 
}

#include <Eigen/Dense>

using namespace Eigen;

void RandomNavigation::moveToDestination() {
    if (!agentNode) return;

    // Obtén la posición actual
    const double* currentPosition = agentNode->getField("translation")->getSFVec3f();
    Vector3d current(currentPosition[0], currentPosition[1], 0.0);

    // Calcula el vector de destino
    Vector3d destination(currentDestination[0], currentDestination[1], 0.0);
    velocity = destination - current;

    #if DEBUG
        std::cout << "destination: [" << destination.data()[0] << " " << destination.data()[1] << " " << destination.data()[2] << "]" << std::endl;
        std::cout << "current: [" << current.data()[0] << " " << current.data()[1] << " " << current.data()[2] << "]" << std::endl;
        std::cout << "movementSpeed: [" << movementSpeed << "]" << std::endl;
        std::cout << "velocity: [" << velocity.data()[0] << " " << velocity.data()[1] << " " << velocity.data()[2] << "]" << std::endl;
    #endif


    // Normaliza la dirección
    double magnitude = velocity.norm();

    #if DEBUG
        std::cout << "magnitude: [" << magnitude << "]" << std::endl;
    #endif

    if (magnitude > 0) {
        velocity.normalize(); // Normaliza el vector

        // Escalar el vector por la velocidad deseada
        velocity *= movementSpeed;
    } else {
        generateRandomDestination(currentDestination);
        return;
    }

    double speed[6] = {velocity.data()[0], velocity.data()[1], velocity.data()[2], 0, 0,0};
    // Llama a setVelocity con el vector de velocidad
    agentNode->setVelocity(speed);

    #if DEBUG
        std::cout << "Setting velocity: [" << velocity.data()[0] << " " << velocity.data()[1] << " " << velocity.data()[2] << "]" << std::endl;
    #endif

}

void RandomNavigation::checkArrival() {

    if(!currentDestination)
        return;
        
    // Aquí resisamos si el agente ha llegado a su destino.
    // Si ha llegado, entonces pide un nuevo destino al Crowd
    if (hasArrivedToDestination()) {

        generateRandomDestination(currentDestination);

        #if DEBUG
            std::cout << "[ " << getName() << "] " << "Destination arrived. New destination: [" << currentDestination[0] << " " << currentDestination[1] << " " << currentDestination[2] << "]" << std::endl;
        #endif
    }
}

bool RandomNavigation::hasArrivedToDestination() {

    if(!currentDestination)
        return false;

    const double* agentPosition = agentNode->getField("translation")->getSFVec3f();

    double distance = sqrt(
            pow(agentPosition[0] - currentDestination[0], 2) +
            pow(agentPosition[1] - currentDestination[1], 2)
    );

    return distance < 0.1;  // Suponiendo 0.1 como un umbral de llegada.
}

void RandomNavigation::setSpeedMovement(float movementSpeed){
    this->movementSpeed = movementSpeed;
}