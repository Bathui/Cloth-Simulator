#include "Cloth.h"
#include <iostream>
#include <glm/gtc/constants.hpp> // For glm::root_two
#include <algorithm> // For std::sort

Cloth::Cloth(int width, int height, float spacing, float totalMass) {
    InitCloth(width, height, spacing, totalMass);
    SetupMesh(); 
}

Cloth::~Cloth() {
    // Clean up dynamically allocated memory
    for (auto p : particles) delete p;
    for (auto s : springs) delete s;
    for (auto t : triangles) delete t;
}

void Cloth::InitCloth(int width, int height, float spacing, float totalMass) {
    m_width = width;
    m_height = height;
    m_spacing = spacing;
    m_totalMass = totalMass;

    // up, down, right and left neighbor particle
    float ksStruct = 450.0f, kdStruct = 0.5f;
    // diagonal particles
    float ksShear  = 100.0f, kdShear  = 0.5f;
    // up, down, right and left one particle
    float ksBend   = 200.0f, kdBend   = 0.5f;

    float particleMass = totalMass / (width * height);

    // 1. GENERATE PARTICLES
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Center the cloth around x=0, and hang it downwards (-y)
            glm::vec3 pos(
                (x - width / 2.0f) * spacing, 
                -y * spacing + 5.0f, // Start a bit high in the air
                sin(x * 0.5f)*0.1f// Add a small curve to break 2D symmetry!
            );

            Particle* p = new Particle(pos, particleMass);
            
            // Fix the top row of particles
            if (y == 0) {
                p->isFixed = true;
            }
            
            particles.push_back(p);
        }
    }
    auto GetParticle = [&](int x, int y) -> Particle* {
        return particles[y * width + x];
    };
    // 2. GENERATE SPRINGS
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Particle* p = GetParticle(x, y);
            // Generate Structural Springs
            if (x < width - 1) springs.push_back(new SpringDamper(p, GetParticle(x + 1, y), ksStruct, kdStruct, spacing));
            if (y < height - 1) springs.push_back(new SpringDamper(p, GetParticle(x, y + 1), ksStruct, kdStruct, spacing));
            
            // Generate Shear Springs
            float shearDist = glm::root_two<float>() * spacing;
            if (x < width - 1 && y < height - 1) {
                springs.push_back(new SpringDamper(p, GetParticle(x + 1, y + 1), ksShear, kdShear, shearDist));
                springs.push_back(new SpringDamper(GetParticle(x + 1, y), GetParticle(x, y + 1), ksShear, kdShear, shearDist));
            }
            // Generate Bending Springs
            if (x < width - 2) springs.push_back(new SpringDamper(p, GetParticle(x + 2, y), ksBend, kdBend, spacing * 2.0f));
            if (y < height - 2) springs.push_back(new SpringDamper(p, GetParticle(x, y + 2), ksBend, kdBend, spacing * 2.0f));
        }
    }
    // 3. GENERATE TRIANGLES AND OPENGL INDICES
    for (int y = 0; y < height - 1; ++y) {
        for (int x = 0; x < width - 1; ++x) {
            // vertices position
            int topLeftIdx     = y * width + x;
            int topRightIdx    = y * width + (x + 1);
            int bottomLeftIdx  = (y + 1) * width + x;
            int bottomRightIdx = (y + 1) * width + (x + 1);
            
            Particle* pTL = particles[topLeftIdx];
            Particle* pTR = particles[topRightIdx];
            Particle* pBL = particles[bottomLeftIdx];
            Particle* pBR = particles[bottomRightIdx];
        
            // Triangle 1
            triangles.push_back(new Triangle(pTL, pBL, pTR));
            indices.push_back(topLeftIdx);
            indices.push_back(bottomLeftIdx);
            indices.push_back(topRightIdx);

            // Triangle 2
            triangles.push_back(new Triangle(pTR, pBL, pBR));
            indices.push_back(topRightIdx);
            indices.push_back(bottomLeftIdx);
            indices.push_back(bottomRightIdx);
        }
    }

    vertexData.resize(particles.size() * 6);
}

void Cloth::UpdatePhysics(float deltaTime, const glm::vec3& windVelocity) {
    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    float airDensity = 1.225f; // Standard air density
    float dragCoefficient = 1.5f; // Fabric drag coefficient

    // 1. Reset normals and forces
    for (Particle* p : particles) {
        p->normal = glm::vec3(0.0f);
        p->ClearForces();
        p->ApplyForce(gravity * p->mass); // Apply Gravity
    }

    // 2. Compute Spring Forces
    for (SpringDamper* sd : springs) {
        sd->ComputeForce();
    }

    // 3. Compute Triangles (Normals and Aerodynamics)
    for (Triangle* t : triangles) {
        t->ComputeNormal();
        t->ComputeAerodynamicForce(windVelocity, airDensity, dragCoefficient);
    }

    // 3.5 Compute Self-Collision
    // Optimized with 1D Sweep and Prune (Sorting along the X axis) to avoid O(N^2) checks.
    float selfCollisionPoints = 0.3f; // Thickness threshold before repulsion
    float kRepel = 2000.0f;           // Stiff repulsion spring
    
    // Create a sorted array of particles along the X axis
    thread_local std::vector<Particle*> sortedParticles;
    if (sortedParticles.size() != particles.size()) {
        sortedParticles = particles;
    } else {
        std::copy(particles.begin(), particles.end(), sortedParticles.begin());
    }
    
    std::sort(sortedParticles.begin(), sortedParticles.end(), [](Particle* a, Particle* b) {
        return a->position.x < b->position.x;
    });

    for (size_t i = 0; i < sortedParticles.size(); ++i) {
        Particle* p1 = sortedParticles[i];
        for (size_t j = i + 1; j < sortedParticles.size(); ++j) {
            Particle* p2 = sortedParticles[j];
            
            // Sweep and Prune: Since the particles are sorted by X, 
            // if the distance in X is greater than the threshold, no particles 
            // further down the array can possibly collide with p1. We can break early!
            if (p2->position.x - p1->position.x > selfCollisionPoints) {
                break;
            }

            // Optimization: Don't compute self-collision for adjacent fixed items
            if (p1->isFixed && p2->isFixed) continue;

            glm::vec3 diff = p1->position - p2->position;
            
            // Check approximate distance with dot product first to avoid square root
            if (glm::dot(diff, diff) < (selfCollisionPoints * selfCollisionPoints)) {
                float dist = glm::length(diff);
                if (dist > 0.0001f /* dist < selfCollisionPoints is already guaranteed by the dot product check */) {
                    glm::vec3 dir = diff / dist;
                    float overlap = selfCollisionPoints - dist;
                    glm::vec3 force = dir * overlap * kRepel;
                    p1->ApplyForce(force);
                    p2->ApplyForce(-force);
                }
            }
        }
    }

    // 4. Normalize Vertices, Integrate, and Handle Ground Collision
    float groundY = -10.0f; // The height of your ground plane
    float groundRestitution = 0.2f; // How bouncy the ground is
    float groundFriction = 0.8f; // How much it slides (0.0 = ice, 1.0 = sticks completely)


    for (Particle* p : particles) {
        // Prepare normals for rendering
        if (glm::length(p->normal) > 0.0f) {
            p->normal = glm::normalize(p->normal);
        } else {
            p->normal = glm::vec3(0.0f, 1.0f, 0.0f); // Fallback
        }

        // Integrate (Update position/velocity)
        p->Update(deltaTime);

        // Ground Plane Collision handling (Added cloth thickness to avoid Z-fighting)
        float clothThickness = 0.05f;
        if (p->position.y < groundY + clothThickness) { // have to check it one more time
            p->position.y = groundY + clothThickness;
            
            // Reflect velocity and apply damping/friction
            p->velocity.y = -p->velocity.y * groundRestitution;
            p->velocity.x *= (1.0f - groundFriction);
            p->velocity.z *= (1.0f - groundFriction);
        } 
    }
}

void Cloth::SetupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glBindVertexArray(0);
}

void Cloth::UpdateMesh() {
    int index = 0;
    for (Particle* p : particles) {
        vertexData[index++] = p->position.x;
        vertexData[index++] = p->position.y;
        vertexData[index++] = p->position.z;
        vertexData[index++] = p->normal.x;
        vertexData[index++] = p->normal.y;
        vertexData[index++] = p->normal.z;
    }

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexData.size() * sizeof(float), vertexData.data());
}

void Cloth::Draw(unsigned int shaderProgram) {
    glUseProgram(shaderProgram);

    UpdateMesh();

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0); 
    glBindVertexArray(0);
}

void Cloth::Reset() {
    // Clean up old data
    for (auto p : particles) delete p;
    for (auto s : springs) delete s;
    for (auto t : triangles) delete t;
    particles.clear();
    springs.clear();
    triangles.clear();
    indices.clear();
    vertexData.clear();

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    // Re-initialize
    InitCloth(m_width, m_height, m_spacing, m_totalMass);
    SetupMesh();
}