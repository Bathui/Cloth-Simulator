#include "ParachuteSystem.h"
#include <algorithm>
#include <cfloat>

ParachuteSystem::ParachuteSystem(glm::vec3 dropPosition) {
    falling = false;
    m_dropPosition = dropPosition;

    // 1. Create canopy cloth — reposition to lay FLAT (X-Z plane) with dome shape
    int gridW = 20, gridH = 20;
    float spacing = 0.8f;
    float canopyMass = 3.0f;
    canopy = new Cloth(gridW, gridH, spacing, canopyMass);

    // Reposition: lay flat in X-Z plane centered at dropPosition, with a dome curve
    for (int gy = 0; gy < gridH; gy++) {
        for (int gx = 0; gx < gridW; gx++) {
            Particle* p = canopy->particles[gy * gridW + gx];
            float dx = (gx - (gridW - 1) / 2.0f) * spacing; // X spread
            float dz = (gy - (gridH - 1) / 2.0f) * spacing;  // Z spread
            // Dome: center is higher, edges lower
            float nx = (gx - (gridW - 1) / 2.0f) / ((gridW - 1) / 2.0f); // -1 to 1
            float nz = (gy - (gridH - 1) / 2.0f) / ((gridH - 1) / 2.0f); // -1 to 1
            float r2 = nx * nx + nz * nz;
            float dome = (1.0f - glm::min(r2, 1.0f)) * 2.0f; // 2 units dome at center
            p->position = dropPosition + glm::vec3(dx, dome, dz);
            p->isFixed = true;
        }
    }

    // Reinforce corner particles at rope attachment points
    float cornerMass = 0.5f;
    canopy->particles[0]->mass = cornerMass;
    canopy->particles[gridW - 1]->mass = cornerMass;
    canopy->particles[gridW * (gridH - 1)]->mass = cornerMass;
    canopy->particles[gridW * (gridH - 1) + gridW - 1]->mass = cornerMass;

    // Stiffen canopy springs for parachute (Scene 1 uses default Cloth values)
    for (auto s : canopy->springs) {
        s->springConstant *= 3.0f;  // Stiff fabric to hold dome shape
        s->dampingFactor  *= 2.0f;
    }

    // 2. Create a heavy crate well below the canopy
    crate = new Cube(dropPosition - glm::vec3(0.0f, 12.0f, 0.0f), 2.0f, 10.0f);
    
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

    // 4 ropes: cloth corner -> crate TOP corner
    int gridW = 20;
    int gridH = 20;
    Particle* clothCorners[4] = {
        canopy->particles[0],
        canopy->particles[gridW - 1],
        canopy->particles[gridW * (gridH - 1)],
        canopy->particles[gridW * (gridH - 1) + gridW - 1]
    };
    // Crate particles 2,3,6,7 are the TOP corners (y = +s)
    Particle* crateCorners[4] = {
        crate->particles[2],
        crate->particles[3],
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

    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    float airDensity = 1.225f;
    float dragCoefficient = 3.0f;  // High drag for parachute canopy
    float velocityDamping = 0.995f;
    float groundY = -10.0f;
    float groundRestitution = 0.3f;
    float groundFriction = 0.8f;

    // ===== PHASE 1: CLEAR ALL FORCES =====
    for (auto p : canopy->particles) {
        p->normal = glm::vec3(0.0f);
        p->ClearForces();
    }
    for (auto p : crate->particles) {
        p->ClearForces();
    }
    for (auto p : ropeParticles) {
        p->ClearForces();
    }

    // ===== PHASE 2: APPLY GRAVITY TO ALL =====
    for (auto p : canopy->particles) p->ApplyForce(gravity * p->mass);
    for (auto p : crate->particles)  p->ApplyForce(gravity * p->mass);
    for (auto p : ropeParticles)     p->ApplyForce(gravity * p->mass);

    // ===== PHASE 3: COMPUTE ALL SPRING FORCES =====
    // Canopy internal springs (structural, shear, bending)
    for (auto sd : canopy->springs) {
        sd->ComputeForce();
    }
    // Crate internal springs (rigidity)
    for (auto s : crate->springs) {
        s->ComputeForce();
    }
    // Rope springs (connect canopy <-> rope particles <-> crate)
    // These now correctly apply forces to canopy and crate particles
    // BEFORE integration, so the coupling is bidirectional.
    for (auto r : ropes) {
        r->ComputeForce();
    }

    // ===== PHASE 4: AERODYNAMIC FORCES ON CANOPY =====
    for (auto t : canopy->triangles) {
        t->ComputeNormal();
        t->ComputeAerodynamicForce(wind, airDensity, dragCoefficient);
    }

    // ===== PHASE 5: CANOPY SELF-COLLISION (position-based) =====
    // Position-based correction is more robust than force-based for preventing penetration
    float selfCollisionThresh = 0.35f;

    thread_local std::vector<Particle*> sortedParticles;
    if (sortedParticles.size() != canopy->particles.size()) {
        sortedParticles = canopy->particles;
    } else {
        std::copy(canopy->particles.begin(), canopy->particles.end(), sortedParticles.begin());
    }
    std::sort(sortedParticles.begin(), sortedParticles.end(), [](Particle* a, Particle* b) {
        return a->position.x < b->position.x;
    });
    for (size_t i = 0; i < sortedParticles.size(); ++i) {
        Particle* p1 = sortedParticles[i];
        for (size_t j = i + 1; j < sortedParticles.size(); ++j) {
            Particle* p2 = sortedParticles[j];
            if (p2->position.x - p1->position.x > selfCollisionThresh) break;
            if (p1->isFixed && p2->isFixed) continue;
            glm::vec3 diff = p1->position - p2->position;
            float dist2 = glm::dot(diff, diff);
            if (dist2 < (selfCollisionThresh * selfCollisionThresh) && dist2 > 0.00001f) {
                float dist = sqrt(dist2);
                glm::vec3 dir = diff / dist;
                float overlap = selfCollisionThresh - dist;

                // Position-based: push particles apart directly
                if (!p1->isFixed && !p2->isFixed) {
                    p1->position += dir * (overlap * 0.5f);
                    p2->position -= dir * (overlap * 0.5f);
                } else if (!p1->isFixed) {
                    p1->position += dir * overlap;
                } else {
                    p2->position -= dir * overlap;
                }

                // Kill approach velocity
                glm::vec3 relVel = p1->velocity - p2->velocity;
                float approach = glm::dot(relVel, dir);
                if (approach < 0.0f) {
                    glm::vec3 impulse = dir * approach * 0.5f;
                    if (!p1->isFixed) p1->velocity -= impulse;
                    if (!p2->isFixed) p2->velocity += impulse;
                }
            }
        }
    }

    // ===== PHASE 6: VELOCITY DAMPING ON ROPES =====
    for (auto p : ropeParticles) {
        p->velocity *= velocityDamping;
    }

    // ===== PHASE 7: CANOPY/ROPE vs CUBE AABB COLLISION =====
    // Compute the crate's axis-aligned bounding box from its particles
    glm::vec3 crateMin(FLT_MAX), crateMax(-FLT_MAX);
    for (auto cp : crate->particles) {
        crateMin = glm::min(crateMin, cp->position);
        crateMax = glm::max(crateMax, cp->position);
    }
    // Add a small margin so particles don't clip through faces
    float margin = 0.15f;
    crateMin -= glm::vec3(margin);
    crateMax += glm::vec3(margin);

    auto resolveAABB = [&](Particle* p) {
        if (p->isFixed) return;
        glm::vec3& pos = p->position;
        // Check if particle is inside the AABB
        if (pos.x > crateMin.x && pos.x < crateMax.x &&
            pos.y > crateMin.y && pos.y < crateMax.y &&
            pos.z > crateMin.z && pos.z < crateMax.z) {
            // Find nearest face to push out along
            float dx1 = pos.x - crateMin.x;
            float dx2 = crateMax.x - pos.x;
            float dy1 = pos.y - crateMin.y;
            float dy2 = crateMax.y - pos.y;
            float dz1 = pos.z - crateMin.z;
            float dz2 = crateMax.z - pos.z;

            float minPen = dx1;
            glm::vec3 pushDir(-1, 0, 0);

            if (dx2 < minPen) { minPen = dx2; pushDir = glm::vec3(1, 0, 0); }
            if (dy1 < minPen) { minPen = dy1; pushDir = glm::vec3(0, -1, 0); }
            if (dy2 < minPen) { minPen = dy2; pushDir = glm::vec3(0, 1, 0); }
            if (dz1 < minPen) { minPen = dz1; pushDir = glm::vec3(0, 0, -1); }
            if (dz2 < minPen) { minPen = dz2; pushDir = glm::vec3(0, 0, 1); }

            // Push particle out
            pos += pushDir * minPen;

            // Kill velocity into the box
            float velInto = glm::dot(p->velocity, -pushDir);
            if (velInto > 0.0f) {
                p->velocity += pushDir * velInto * 1.1f; // Slight bounce
            }
        }
    };

    for (auto p : canopy->particles) resolveAABB(p);
    for (auto p : ropeParticles) resolveAABB(p);

    // ===== PHASE 8: CLAMP ACCELERATION (safety net) =====
    float maxAccel = 2000.0f;
    auto clampForce = [maxAccel](Particle* p) {
        if (p->isFixed || p->mass <= 0.0f) return;
        glm::vec3 accel = p->forceAccumulator / p->mass;
        float accelMag = glm::length(accel);
        if (accelMag > maxAccel) {
            p->forceAccumulator = glm::normalize(accel) * maxAccel * p->mass;
        }
    };
    for (auto p : canopy->particles) clampForce(p);
    for (auto p : crate->particles)  clampForce(p);
    for (auto p : ropeParticles)     clampForce(p);

    // ===== PHASE 9: INTEGRATE ALL PARTICLES =====
    // Canopy particles
    for (auto p : canopy->particles) {
        if (glm::length(p->normal) > 0.0f) {
            p->normal = glm::normalize(p->normal);
        } else {
            p->normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        p->Update(deltaTime);
        if (p->position.y < groundY + 0.05f) {
            p->position.y = groundY + 0.05f;
            p->velocity.y = -p->velocity.y * groundRestitution;
            p->velocity.x *= (1.0f - groundFriction);
            p->velocity.z *= (1.0f - groundFriction);
        }
    }
    // Crate particles
    for (auto p : crate->particles) {
        p->Update(deltaTime);
        if (p->position.y < groundY) {
            p->position.y = groundY;
            p->velocity.y = -p->velocity.y * groundRestitution;
            p->velocity.x *= (1.0f - groundFriction);
            p->velocity.z *= (1.0f - groundFriction);
        }
    }
    // Rope particles
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

    int gridW = 20, gridH = 20;
    float spacing = 0.8f;
    canopy = new Cloth(gridW, gridH, spacing, 3.0f);
    for (int gy = 0; gy < gridH; gy++) {
        for (int gx = 0; gx < gridW; gx++) {
            Particle* p = canopy->particles[gy * gridW + gx];
            float dx = (gx - (gridW - 1) / 2.0f) * spacing;
            float dz = (gy - (gridH - 1) / 2.0f) * spacing;
            float nx = (gx - (gridW - 1) / 2.0f) / ((gridW - 1) / 2.0f);
            float nz = (gy - (gridH - 1) / 2.0f) / ((gridH - 1) / 2.0f);
            float r2 = nx * nx + nz * nz;
            float dome = (1.0f - glm::min(r2, 1.0f)) * 2.0f;
            p->position = m_dropPosition + glm::vec3(dx, dome, dz);
            p->isFixed = true;
        }
    }

    // Reinforce corners (same as constructor)
    float cornerMass = 0.5f;
    canopy->particles[0]->mass = cornerMass;
    canopy->particles[gridW - 1]->mass = cornerMass;
    canopy->particles[gridW * (gridH - 1)]->mass = cornerMass;
    canopy->particles[gridW * (gridH - 1) + gridW - 1]->mass = cornerMass;

    // Stiffen canopy springs (same as constructor)
    for (auto s : canopy->springs) {
        s->springConstant *= 3.0f;
        s->dampingFactor  *= 2.0f;
    }

    crate = new Cube(m_dropPosition - glm::vec3(0.0f, 12.0f, 0.0f), 2.0f, 10.0f);
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