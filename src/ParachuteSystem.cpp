#include "ParachuteSystem.h"

ParachuteSystem::ParachuteSystem(glm::vec3 dropPosition) {
    falling = false;
    m_dropPosition = dropPosition;

    // 1. Create a lightweight cloth high in the air
    canopy = new Cloth(20, 20, 0.4f, 1.0f); 
    
    // Shift canopy to drop position and FIX all particles (frozen until space)
    for(auto p : canopy->particles) {
        p->position += dropPosition;
        p->isFixed = true; 
    }

    // 2. Create a heavy crate below the canopy
    crate = new Cube(dropPosition - glm::vec3(0.0f, 6.0f, 0.0f), 2.0f, 10.0f);
    
    // Fix all crate particles too (frozen until space)
    for(auto p : crate->particles) {
        p->isFixed = true;
    }

    // 3. Create rope chains
    CreateRopes();

    // 4. Initialize OpenGL buffers for rendering the lines
    SetupLineMesh();
}

ParachuteSystem::~ParachuteSystem() {
    delete canopy;
    delete crate;
    for (auto r : ropes) delete r;
    for (auto p : ropeParticles) delete p;
    
    glDeleteVertexArrays(1, &lineVAO);
    glDeleteBuffers(1, &lineVBO);
}

void ParachuteSystem::CreateRopes() {
    float ropeKs = 500.0f;   // Moderate stiffness (lowered from 1500 to prevent blow-up)
    float ropeKd = 20.0f;    // Good damping on the spring itself
    int segments = 8;
    float ropeMassPerParticle = 0.1f; // Heavier particles = more stable

    // 4 ropes: cloth corner -> crate top corner
    Particle* clothCorners[4] = {
        canopy->particles[0],
        canopy->particles[19],
        canopy->particles[20 * 19],
        canopy->particles[20 * 19 + 19]
    };
    Particle* crateCorners[4] = {
        crate->particles[4],
        crate->particles[5],
        crate->particles[6],
        crate->particles[7]
    };

    for (int r = 0; r < 4; r++) {
        Particle* start = clothCorners[r];
        Particle* end = crateCorners[r];

        Particle* prev = start;
        float segmentLength = glm::distance(start->position, end->position) / segments;

        for (int i = 1; i < segments; i++) {
            float t = (float)i / (float)segments;
            glm::vec3 pos = glm::mix(start->position, end->position, t);
            Particle* p = new Particle(pos, ropeMassPerParticle);
            p->isFixed = !falling;
            ropeParticles.push_back(p);

            ropes.push_back(new SpringDamper(prev, p, ropeKs, ropeKd, segmentLength));
            prev = p;
        }

        ropes.push_back(new SpringDamper(prev, end, ropeKs, ropeKd, segmentLength));
    }
}

void ParachuteSystem::UpdatePhysics(float deltaTime, const glm::vec3& wind) {
    // Don't simulate if not yet released
    if (!falling) return;

    // Compute internal cloth forces and crate forces
    canopy->UpdatePhysics(deltaTime, wind);
    crate->UpdatePhysics(deltaTime);

    // Apply gravity, velocity damping, and clear forces on rope particles
    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    float velocityDamping = 0.995f;
    for (auto p : ropeParticles) {
        p->ClearForces();
        p->ApplyForce(gravity * p->mass);
        p->velocity *= velocityDamping;
    }

    // Compute forces for the suspension rope springs
    for(auto r : ropes) {
        r->ComputeForce();
    }

    // --- Inter-object collision (position-based correction) ---
    // Instead of applying forces (which can blow up), directly push particles apart
    float collisionRadius = 0.4f;

    auto resolveCollision = [&](std::vector<Particle*>& setA, std::vector<Particle*>& setB) {
        for (auto pA : setA) {
            for (auto pB : setB) {
                glm::vec3 diff = pA->position - pB->position;
                float dist2 = glm::dot(diff, diff);
                if (dist2 < collisionRadius * collisionRadius && dist2 > 0.00001f) {
                    float dist = sqrt(dist2);
                    glm::vec3 dir = diff / dist;
                    float overlap = collisionRadius - dist;

                    // Position correction: push each particle half the overlap distance
                    if (!pA->isFixed && !pB->isFixed) {
                        pA->position += dir * (overlap * 0.5f);
                        pB->position -= dir * (overlap * 0.5f);
                    } else if (!pA->isFixed) {
                        pA->position += dir * overlap;
                    } else if (!pB->isFixed) {
                        pB->position -= dir * overlap;
                    }

                    // Dampen approach velocity (kill the component moving them together)
                    glm::vec3 relVel = pA->velocity - pB->velocity;
                    float approachSpeed = glm::dot(relVel, dir);
                    if (approachSpeed < 0.0f) { // Only if moving toward each other
                        glm::vec3 impulse = dir * approachSpeed * 0.5f;
                        if (!pA->isFixed) pA->velocity -= impulse;
                        if (!pB->isFixed) pB->velocity += impulse;
                    }
                }
            }
        }
    };

    // Canopy vs Crate
    resolveCollision(canopy->particles, crate->particles);
    // Rope vs Crate
    resolveCollision(ropeParticles, crate->particles);

    // Integrate rope particles
    float groundY = -10.0f;
    for (auto p : ropeParticles) {
        p->Update(deltaTime);
        if (p->position.y < groundY) {
            p->position.y = groundY;
            p->velocity.y = -p->velocity.y * 0.3f;
        }
    }
}

void ParachuteSystem::Reset() {
    // Clean up old data
    delete canopy;
    delete crate;
    for (auto r : ropes) delete r;
    ropes.clear();
    for (auto p : ropeParticles) delete p;
    ropeParticles.clear();
    glDeleteVertexArrays(1, &lineVAO);
    glDeleteBuffers(1, &lineVBO);

    // Rebuild everything from scratch
    falling = false;

    canopy = new Cloth(20, 20, 0.4f, 1.0f);
    for(auto p : canopy->particles) {
        p->position += m_dropPosition;
        p->isFixed = true;
    }

    crate = new Cube(m_dropPosition - glm::vec3(0.0f, 6.0f, 0.0f), 2.0f, 10.0f);
    for(auto p : crate->particles) {
        p->isFixed = true;
    }

    CreateRopes();
    SetupLineMesh();
}

void ParachuteSystem::StartFalling() {
    if (falling) return;
    falling = true;
    
    // Unpin all canopy particles
    for(auto p : canopy->particles) {
        p->isFixed = false;
    }
    // Unpin all crate particles 
    for(auto p : crate->particles) {
        p->isFixed = false;
    }
    // Unpin all rope particles
    for(auto p : ropeParticles) {
        p->isFixed = false;
    }
}

void ParachuteSystem::SetupLineMesh() {
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    
    // Allocate enough space for all rope segments (2 vertices per line, 6 floats per vertex)
    size_t maxLines = ropes.size();
    glBufferData(GL_ARRAY_BUFFER, maxLines * 2 * 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

    // Position attribute (Location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Normal attribute (Location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void ParachuteSystem::DrawLines(unsigned int shaderProgram) {
    lineVertexData.clear();

    auto pushSpringLine = [&](SpringDamper* s) {
        lineVertexData.push_back(s->p1->position.x);
        lineVertexData.push_back(s->p1->position.y);
        lineVertexData.push_back(s->p1->position.z);
        lineVertexData.push_back(0.0f); lineVertexData.push_back(1.0f); lineVertexData.push_back(0.0f); 
        
        lineVertexData.push_back(s->p2->position.x);
        lineVertexData.push_back(s->p2->position.y);
        lineVertexData.push_back(s->p2->position.z);
        lineVertexData.push_back(0.0f); lineVertexData.push_back(1.0f); lineVertexData.push_back(0.0f); 
    };

    for (auto r : ropes) pushSpringLine(r);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, lineVertexData.size() * sizeof(float), lineVertexData.data());

    glBindVertexArray(lineVAO);
    glDrawArrays(GL_LINES, 0, lineVertexData.size() / 6);
    glBindVertexArray(0);
}

void ParachuteSystem::DrawCrate(unsigned int shaderProgram) {
    crate->Draw(shaderProgram);
}