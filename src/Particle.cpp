#include "Particle.h"

Particle::Particle(glm::vec3 initialPosition, float particleMass) {
    position = initialPosition;
    velocity = glm::vec3(0.0f);
    forceAccumulator = glm::vec3(0.0f);
    normal = glm::vec3(0.0f, 1.0f, 0.0f); // Default pointing up
    mass = particleMass;
    isFixed = false;
}

void Particle::ApplyForce(const glm::vec3& force) {
    forceAccumulator += force;
}

void Particle::ClearForces() {
    forceAccumulator = glm::vec3(0.0f);
}

void Particle::Update(float deltaTime) {
    // If the particle is pinned (like the top corners of the cloth)
    // or has no mass, it shouldn't move.
    if (isFixed || mass <= 0.0f) {
        return;
    }

    // 1. Calculate acceleration from accumulated forces (a = F/m)
    glm::vec3 acceleration = forceAccumulator / mass;

    // 2. Semi-Implicit Euler Integration
    // Update velocity FIRST
    velocity += acceleration * deltaTime;

    // Update position SECOND using the new velocity
    position += velocity * deltaTime;
}