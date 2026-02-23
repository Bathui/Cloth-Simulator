#pragma once

#include <glm/glm.hpp>

class Particle {
public:
    // Core physical properties
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 forceAccumulator; 
    
    // Properties for rendering and aerodynamics
    glm::vec3 normal; 
    
    float mass;
    bool isFixed; // If true, the particle ignores forces and integration

    // Constructor
    Particle(glm::vec3 initialPosition, float particleMass);

    // Physics methods
    void ApplyForce(const glm::vec3& force);
    void ClearForces();
    
    // Integrates the accumulated forces to update velocity and position
    void Update(float deltaTime); 
};