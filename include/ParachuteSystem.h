#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

#include "Cloth.h"
#include "Cube.h"
#include "SpringDamper.h"

class ParachuteSystem {
public:
    Cloth* canopy;
    Cube* crate;
    std::vector<SpringDamper*> ropes;
    std::vector<Particle*> ropeParticles; // Intermediate particles along each rope chain
    bool falling;
    glm::vec3 m_dropPosition;

    // OpenGL Line Rendering state for ropes
    unsigned int lineVAO, lineVBO;
    std::vector<float> lineVertexData;

    // Constructor & Destructor
    ParachuteSystem(glm::vec3 dropPosition);
    ~ParachuteSystem();

    // Core Functions
    void UpdatePhysics(float deltaTime, const glm::vec3& wind);
    void StartFalling();
    void Reset();
    void CreateRopes(); // Helper to build rope chains
    void SetupLineMesh();
    void DrawLines(unsigned int shaderProgram);
    void DrawCrate(unsigned int shaderProgram);
};