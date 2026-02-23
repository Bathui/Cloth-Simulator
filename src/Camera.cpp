#include "Camera.h"


Camera::Camera(glm::vec3 position) : 
        Front(glm::vec3(0.0f, 0.0f, -1.0f)), 
        MovementSpeed(2.5f), 
        MouseSensitivity(0.1f), 
        Zoom(45.0f) 
    {
        Position = position;
        WorldUp = glm::vec3(0.0f, 1.0f, 0.0f);
        Yaw = -90.0f;
        Pitch = 0.0f;
        updateCameraVectors();
}

void Camera::ProcessKeyboard(int direction, float deltaTime) {
    float velocity = MovementSpeed * deltaTime;
    if (direction == 0) Position += Front * velocity; // FORWARD
    if (direction == 1) Position -= Front * velocity; // BACKWARD
    if (direction == 2) Position -= Right * velocity; // LEFT
    if (direction == 3) Position += Right * velocity; // RIGHT
    if (direction == 4) Position += Up * velocity;    // UP
    if (direction == 5) Position -= Up * velocity;    // DOWN
}

void Camera::updateCameraVectors() {
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);
    
    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up    = glm::normalize(glm::cross(Right, Front));
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw   += xoffset;
    Pitch += yoffset;

    // Constrain pitch to avoid screen flip
    if (Pitch > 89.0f) Pitch = 89.0f;
    if (Pitch < -89.0f) Pitch = -89.0f;

    updateCameraVectors();
}