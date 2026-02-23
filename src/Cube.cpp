#include "Cube.h"

Cube::Cube(glm::vec3 center, float size, float mass){
    float s = size / 2.0f;
    float pMass = mass / 8.0f;
    float ks = 5000.0f; // Very stiff for a "solid" feel
    float kd = 50.0f;   // High damping to prevent jitter

    // 1. Create 8 corners
    // Order: z=0 face first, then z=1 face
    // z=0: (0)---x, (1)+x, (2)---x+y, (3)+x+y
    // z=1: (4)---x, (5)+x, (6)---x+y, (7)+x+y
    for (int z = 0; z < 2; z++) {
        for (int y = 0; y < 2; y++) {
            for (int x = 0; x < 2; x++) {
                glm::vec3 pos = center + glm::vec3(x ? s : -s, y ? s : -s, z ? s : -s);
                particles.push_back(new Particle(pos, pMass));
            }
        }
    }

    // 2. Connect every particle to every other particle to ensure total rigidity
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            float dist = glm::distance(particles[i]->position, particles[j]->position);
            springs.push_back(new SpringDamper(particles[i], particles[j], ks, kd, dist));
        }
    }

    // 3. Define indices for the 6 faces (12 triangles)
    // Particle layout (x,y,z): 0(-,-,-), 1(+,-,-), 2(-,+,-), 3(+,+,-),
    //                           4(-,-,+), 5(+,-,+), 6(-,+,+), 7(+,+,+)
    indices = {
        // Front face (z = -s): 0, 1, 2, 3
        0, 2, 1,  1, 2, 3,
        // Back face (z = +s): 4, 5, 6, 7
        4, 5, 6,  5, 7, 6,
        // Left face (x = -s): 0, 2, 4, 6
        0, 4, 2,  2, 4, 6,
        // Right face (x = +s): 1, 3, 5, 7
        1, 3, 5,  3, 7, 5,
        // Top face (y = +s): 2, 3, 6, 7
        2, 6, 3,  3, 6, 7,
        // Bottom face (y = -s): 0, 1, 4, 5
        0, 1, 4,  1, 5, 4
    };

    vertexData.resize(8 * 6); // 8 vertices, 6 floats each (pos + normal)
    SetupMesh();
}

void Cube::UpdatePhysics(float deltaTime) {
    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    
    // Apply gravity
    for (Particle* p : particles) {
        p->ClearForces();
        p->ApplyForce(gravity * p->mass);
    }

    // Compute spring forces
    for (SpringDamper* s : springs) {
        s->ComputeForce();
    }

    // Integrate and handle ground collision (similar to your cloth logic)
    float groundY = -10.0f;
    float groundRestitution = 0.3f; // Less bouncy than cloth
    float groundFriction = 0.8f;

    for (Particle* p : particles) {
        p->Update(deltaTime);
        if (p->position.y < groundY) {
            p->position.y = groundY;
            p->velocity.y = -p->velocity.y * groundRestitution;
            p->velocity.x *= (1.0f - groundFriction);
            p->velocity.z *= (1.0f - groundFriction);
        }
    }
}

void Cube::SetupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Position attribute (Location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute (Location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void Cube::UpdateMesh() {
    // Compute per-vertex normals by averaging face normals
    // First, reset all normals
    glm::vec3 normals[8] = {};

    // Process each triangle and accumulate face normal to its vertices
    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int i0 = indices[i];
        unsigned int i1 = indices[i + 1];
        unsigned int i2 = indices[i + 2];

        glm::vec3 v0 = particles[i0]->position;
        glm::vec3 v1 = particles[i1]->position;
        glm::vec3 v2 = particles[i2]->position;

        glm::vec3 faceNormal = glm::cross(v1 - v0, v2 - v0);
        normals[i0] += faceNormal;
        normals[i1] += faceNormal;
        normals[i2] += faceNormal;
    }

    // Normalize and pack into vertex data
    int idx = 0;
    for (int i = 0; i < 8; i++) {
        vertexData[idx++] = particles[i]->position.x;
        vertexData[idx++] = particles[i]->position.y;
        vertexData[idx++] = particles[i]->position.z;

        glm::vec3 n = glm::length(normals[i]) > 0.0f ? glm::normalize(normals[i]) : glm::vec3(0.0f, 1.0f, 0.0f);
        vertexData[idx++] = n.x;
        vertexData[idx++] = n.y;
        vertexData[idx++] = n.z;
    }

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexData.size() * sizeof(float), vertexData.data());
}

void Cube::Draw(unsigned int shaderProgram) {
    glUseProgram(shaderProgram);
    UpdateMesh();
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}