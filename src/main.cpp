#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Shader.h"
#include "Camera.h"
#include "Cloth.h"
#include "ParachuteSystem.h" // Includes the new scene

// ImGui Headers
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// Globals for camera control and state
Camera camera(glm::vec3(0.0f, 5.0f, 15.0f));
float lastX = 400, lastY = 300;
bool firstMouse = true;
float deltaTime = 0.0f;
float lastFrame = 0.0f;

bool dropCloth = false;
bool dropParachute = false;
bool rKeyWasPressed = false;
int currentScene = 1; // 1 = Cloth, 2 = Parachute

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn) {
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) != GLFW_PRESS) {
        firstMouse = true;
        return;
    }

    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(0, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(1, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(2, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(3, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        if (currentScene == 1)
            dropCloth = true;
        else if (currentScene == 2)
            dropParachute = true;
    }
        
    // Scene switching keys
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
        currentScene = 1;
    if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
        currentScene = 2;
}

int main () {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "Physics Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);

    //check if the graphic card works
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    glEnable(GL_DEPTH_TEST);
    
    // --- Setup Dear ImGui context ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    // 1. Initialize Shaders and Scenes
    Shader clothShader("Shader/cloth.vert", "Shader/cloth.frag");
    
    // Scene 1: Width (nodes), Height (nodes), Spacing, Total Mass
    Cloth myCloth(20, 20, 0.4f, 2.0f);

    // Scene 2: Parachute System
    ParachuteSystem myParachute(glm::vec3(0.0f, 25.0f, 0.0f));

    glm::vec3 wind(0.0f, 0.0f, 0.0f); // A gentle breeze blowing back

    // --- Ground Plane Setup ---
    float groundVertices[] = {
        // Positions            // Normals
        -50.0f, -10.0f, -50.0f,   0.0f, 1.0f, 0.0f,
         50.0f, -10.0f, -50.0f,   0.0f, 1.0f, 0.0f,
         50.0f, -10.0f,  50.0f,   0.0f, 1.0f, 0.0f,
         50.0f, -10.0f,  50.0f,   0.0f, 1.0f, 0.0f,
        -50.0f, -10.0f,  50.0f,   0.0f, 1.0f, 0.0f,
        -50.0f, -10.0f, -50.0f,   0.0f, 1.0f, 0.0f
    };
    unsigned int groundVAO, groundVBO;
    glGenVertexArrays(1, &groundVAO);
    glGenBuffers(1, &groundVBO);
    
    glBindVertexArray(groundVAO);
    glBindBuffer(GL_ARRAY_BUFFER, groundVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(groundVertices), groundVertices, GL_STATIC_DRAW);
    
    // Position attribute (Location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Normal attribute (Location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glBindVertexArray(0);

    // --- ImGui State Variables ---
    float windSpeedBase = 0.f;
    float windSpeedVariance = 0.0f;
    float swirlSpeed = 0.f;
    float turbulenceStrength = 0.0f;

    // --- Cloth Pin Selection (Grid Coordinates 0 to 19) ---
    int pinLeftX = 0;
    int pinLeftY = 0;
    int pinRightX = 19;
    int pinRightY = 0;

    //----------------------------------------------------------
    // 2. Main Render Loop
    while (!glfwWindowShouldClose(window)) {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        processInput(window);

        // Reset with R key (one-shot: only triggers on initial press)
        bool rKeyDown = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        if (rKeyDown && !rKeyWasPressed) {
            if (currentScene == 1) {
                myCloth.Reset();
                dropCloth = false;
            } else if (currentScene == 2) {
                myParachute.Reset();
                dropParachute = false;
            }
        }
        rKeyWasPressed = rKeyDown;

        // --- Start ImGui Frame ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Sub-stepping the physics for stability (Standard practice for Mass-Spring systems)
        // More substeps = smaller dt = more stable. Cap deltaTime to avoid explosion on first frame.
        if (deltaTime > 0.033f) deltaTime = 0.033f; // Cap at ~30fps worth of time
        
        // --- ImGui Wind Controls Window ---
        ImGui::Begin("Simulation Controls");
        
        ImGui::Text("Current Scene: %d", currentScene);
        ImGui::Text("Press '1' for Cloth, '2' for Parachute");
        ImGui::Separator();

        ImGui::Text("Wind Options");
        ImGui::SliderFloat("Base Speed", &windSpeedBase, 0.0f, 10.0f);
        ImGui::SliderFloat("Speed Variance", &windSpeedVariance, 0.0f, 10.0f);
        ImGui::SliderFloat("Swirl Speed", &swirlSpeed, 0.0f, 5.0f);
        ImGui::SliderFloat("Turbulence", &turbulenceStrength, 0.0f, 5.0f);
        
        ImGui::Separator();
        ImGui::Text("Scene 1 Pinned Particles (Grid X, Y)");
        ImGui::SliderInt("Pin 1 X", &pinLeftX, 0, 19);
        ImGui::SliderInt("Pin 1 Y", &pinLeftY, 0, 19);
        ImGui::SliderInt("Pin 2 X", &pinRightX, 0, 19);
        ImGui::SliderInt("Pin 2 Y", &pinRightY, 0, 19);
        if (ImGui::Button(dropCloth ? "Reset Cloth (Pin Again)" : "Drop Cloth (Spacebar)")) {
            dropCloth = !dropCloth;
        }
        ImGui::End();

        // --- Apply Pin Selection (Only relevant for Scene 1) ---
        if (currentScene == 1) {
            // 1. Unpin everything
            for (Particle* p : myCloth.particles) {
                p->isFixed = false;
            }
            // 2. Pin the individually selected particles if not dropped
            if (!dropCloth) {
                int idx1 = pinLeftY * 20 + pinLeftX;
                int idx2 = pinRightY * 20 + pinRightX;
                if (idx1 >= 0 && idx1 < myCloth.particles.size()) myCloth.particles[idx1]->isFixed = true;
                if (idx2 >= 0 && idx2 < myCloth.particles.size()) myCloth.particles[idx2]->isFixed = true;
            }
        }
        
        // --- Natural Dynamic Wind Simulation ---
        float time = glfwGetTime();
        // Use overlapping sine waves with different frequencies to create a 
        // pseudo-random wind vector that smoothly swirls in all 3D directions.
        float windSpeed = 0.1f * windSpeedBase + sin(time * 0.4f) * windSpeedVariance;
        
        // Slowly rotating primary direction
        float angleXZ = time * swirlSpeed + sin(time * 0.2f); // Swirls around the Y axis
        float angleY  = sin(time * 0.3f) * 0.5f;              // Slight updrafts/downdrafts
        
        glm::vec3 wind(
            sin(angleXZ) * cos(angleY),
            sin(angleY),
            cos(angleXZ) * cos(angleY)
        );
        wind *= windSpeed; // Apply strength to direction
        
        // Add fast, chaotic high-frequency turbulence
        wind += glm::vec3(
            sin(time * 2.1f),
            cos(time * 2.7f),
            sin(time * 3.3f)
        ) * turbulenceStrength;

        // --- Trigger parachute drop if space was pressed in scene 2 ---
        if (dropParachute && currentScene == 2) {
            myParachute.StartFalling();
        }

        // --- Physics Integration ---
        int subSteps = 30;
        float subDeltaTime = deltaTime / subSteps;
        for(int i = 0; i < subSteps; i++) {
            if (currentScene == 1) {
                myCloth.UpdatePhysics(subDeltaTime, wind);
            } else if (currentScene == 2) {
                myParachute.UpdatePhysics(subDeltaTime, wind);
            }
        }

        // Render Background (Grey to match screenshot)
        glClearColor(0.4f, 0.4f, 0.45f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Activate Shader & Set shared camera uniforms
        clothShader.use();
        clothShader.setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.5f));
        clothShader.setVec3("viewPos", camera.Position);

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), 800.0f / 600.0f, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 model = glm::mat4(1.0f); // Identity matrix so everything renders at its actual coordinates
    
        clothShader.setMat4("projection", projection);
        clothShader.setMat4("view", view);
        clothShader.setMat4("model", model);

        // 1. DRAW GROUND
        clothShader.setVec3("objectColor", glm::vec3(0.85f, 0.85f, 0.82f)); 
        glBindVertexArray(groundVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        // 2. DRAW ACTIVE SCENE
        if (currentScene == 1) {
            clothShader.setVec3("objectColor", glm::vec3(0.55f, 0.15f, 0.15f)); 
            myCloth.Draw(clothShader.ID);
        } 
        else if (currentScene == 2) {
            // Parachute Canopy (Green)
            clothShader.setVec3("objectColor", glm::vec3(0.15f, 0.55f, 0.15f)); 
            myParachute.canopy->Draw(clothShader.ID);

            // Ropes (Dark Grey/Black lines)
            clothShader.setVec3("objectColor", glm::vec3(0.1f, 0.1f, 0.1f)); 
            myParachute.DrawLines(clothShader.ID);

            // Crate (Solid brown box)
            clothShader.setVec3("objectColor", glm::vec3(0.55f, 0.35f, 0.15f)); 
            myParachute.DrawCrate(clothShader.ID);
        }

        // 3. RENDER IMGUI
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // --- Cleanup ImGui ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}