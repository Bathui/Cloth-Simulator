#pragma once

#include <glm/glm.hpp>

// Forward declaration of Particle to avoid circular includes
class Particle; 

class SpringDamper {
public:
    // Pointers to the two particles connected by this spring
    Particle* p1;
    Particle* p2;

    // Spring properties
    float springConstant; // ks: stiffness of the spring
    float dampingFactor;  // kd: how quickly it loses energy
    float restLength;     // L0: the length the spring "wants" to be

    // Constructor
    SpringDamper(Particle* particle1, Particle* particle2, float ks, float kd, float initialLength);

    // Calculates the forces and applies them to p1 and p2
    void ComputeForce(); 
};