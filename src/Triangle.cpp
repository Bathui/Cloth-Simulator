#include "Triangle.h"
#include "Particle.h"

Triangle::Triangle(Particle* particle1, Particle* particle2, Particle* particle3) {
    p1 = particle1;
    p2 = particle2;
    p3 = particle3;
}

void Triangle::ComputeNormal() {
    // Calculate two edge vectors
    glm::vec3 e1 = p2->position - p1->position;
    glm::vec3 e2 = p3->position - p1->position;

    // The cross product gives us a vector perpendicular to both edges (the face normal).
    glm::vec3 crossProduct = glm::cross(e1, e2);

    p1->normal += crossProduct;
    p2->normal += crossProduct;
    p3->normal += crossProduct;
}

void Triangle::ComputeAerodynamicForce(const glm::vec3& windVelocity, float airDensity, float dragCoefficient) {
    // 1. Calculate the average velocity of the triangle's surface
    glm::vec3 surfaceVelocity = (p1->velocity + p2->velocity + p3->velocity) / 3.0f;

    // 2. Calculate the relative velocity between the triangle and the wind
    glm::vec3 v_rel = surfaceVelocity - windVelocity;
    float v_rel_length = glm::length(v_rel);

    // If there is no relative motion, there is no drag
    if (v_rel_length == 0.0f) return;

    // 3. Calculate the normal and area
    glm::vec3 e1 = p2->position - p1->position;
    glm::vec3 e2 = p3->position - p1->position;
    glm::vec3 crossProduct = glm::cross(e1, e2);

    // Area of triangle is half the magnitude of the cross product
    float area = glm::length(crossProduct) * 0.5f;

    // Safety check for degenerate (zero-area) triangles
    if (area == 0.0f) return;

    glm::vec3 normal = glm::normalize(crossProduct); // Normalized normal
    // 4. Calculate the aerodynamic drag force
    // We use the dot product to find how much the triangle is facing the wind
    float dotProduct = glm::dot(v_rel, normal);
    float forceMagnitude = -0.5f * airDensity * (v_rel_length * v_rel_length) * dragCoefficient * area * (dotProduct / v_rel_length);

    glm::vec3 aeroForce = forceMagnitude * normal;
    // 5. Distribute the total aerodynamic force equally among the three vertices
    glm::vec3 forcePerParticle = aeroForce / 3.0f;

    p1->ApplyForce(forcePerParticle);
    p2->ApplyForce(forcePerParticle);
    p3->ApplyForce(forcePerParticle);

}