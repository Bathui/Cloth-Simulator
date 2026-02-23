// Cube.h (Logic for the solid crate)
#pragma once
#include <vector>
#include <glad/glad.h>
#include "Particle.h"
#include "SpringDamper.h"

class Cube {
public:
    std::vector<Particle*> particles;
    std::vector<SpringDamper*> springs;

    // OpenGL rendering state for solid faces
    unsigned int VAO, VBO, EBO;
    std::vector<float> vertexData;
    std::vector<unsigned int> indices;

    Cube(glm::vec3 center, float size, float mass);

    ~Cube() {
        for (auto p : particles) delete p;
        for (auto s : springs) delete s;
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }

    void UpdatePhysics(float deltaTime);
    void Draw(unsigned int shaderProgram);

private:
    void SetupMesh();
    void UpdateMesh();
};