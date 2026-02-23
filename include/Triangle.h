#pragma once

#include <glm/glm.hpp>

class Particle;

class Triangle {
public:
    // Pointers to the three vertices (particles) of the triangle
    Particle* p1;
    Particle* p2;
    Particle* p3;

    // Constructor
    Triangle(Particle* particle1, Particle* particle2, Particle* particle3);

    // Calculates the unnormalized face normal and adds it to the particles for smooth shading
    void ComputeNormal();

    // Calculates and applies aerodynamic drag forces based on wind
    void ComputeAerodynamicForce(const glm::vec3& windVelocity, float airDensity, float dragCoefficient);
};