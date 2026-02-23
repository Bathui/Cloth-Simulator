#pragma once

#include <vector>
#include <glad/glad.h>
#include "Particle.h"
#include "SpringDamper.h"
#include "Triangle.h"

class Cloth {
public:
    std::vector<Particle*> particles;
    std::vector<SpringDamper*> springs;
    std::vector<Triangle*> triangles;

    int m_width;
    int m_height;
    float m_spacing;
    float m_totalMass;

    // OpenGL specific data
    std::vector<float> vertexData;     // Stores alternating PosX, PosY, PosZ, NormX, NormY, NormZ
    std::vector<unsigned int> indices; // Defines which vertices make up which triangles

    unsigned int VAO, VBO, EBO;

    Cloth(int width, int height, float spacing, float totalMass);
    ~Cloth();

    void InitCloth(int width, int height, float spacing, float totalMass);
    void UpdatePhysics(float deltaTime, const glm::vec3& windVelocity);
    void Draw(unsigned int shaderProgram);
    void Reset();

private:
    void SetupMesh();
    void UpdateMesh();
};