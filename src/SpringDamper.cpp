#include "SpringDamper.h"
#include "Particle.h"

SpringDamper::SpringDamper(Particle* particle1, Particle* particle2, float ks, float kd, float initialLength) {
    p1 = particle1;
    p2 = particle2;
    springConstant = ks;
    dampingFactor = kd;
    restLength = initialLength;
}

void SpringDamper::ComputeForce() {
    // Safety check
    if (!p1 || !p2) return;

    // 1. Find the distance and direction between the two particles
    glm::vec3 e = p2->position - p1->position;
    float l = glm::length(e);

    // Prevent division by zero if particles occupy the exact same space
    if (l == 0.0f) return;

    // Normalized direction vector from p1 to p2
    glm::vec3 e_hat = e / l;

    // relative velocity
    glm::vec3 v_rel = p2->velocity - p1->velocity;

    float v_rel_1D = glm::dot(v_rel, e_hat);

    float springForceScalar = springConstant * (l - restLength);
    float dampingForceScalar = dampingFactor * v_rel_1D;

    // get our total force to p1
    glm::vec3 f_total = (springForceScalar + dampingForceScalar) * e_hat;

    p1->ApplyForce(f_total);
    p2->ApplyForce(-f_total);
}