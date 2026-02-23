cmake_minimum_required(VERSION 3.15)

# Name your project and specify that it uses both C and C++ (since glad is .c)
project(ClothSimulation C CXX)

# Require modern C++ 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Include Directories
# Tells the compiler where to find <glad/glad.h>, <GLFW/glfw3.h>, <glm/glm.hpp>, etc.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 2. Find OpenGL on your system
find_package(OpenGL REQUIRED)

# 3. Gather all your source files
# Note: If you add the ImGui .cpp files later, you must add them to this list!
set(SOURCES
    src/main.cpp
    src/Cloth.cpp
    src/Particle.cpp
    src/SpringDamper.cpp
    src/Triangle.cpp
    src/Camera.cpp
    src/ParachuteSystem.cpp
    src/Cube.cpp
    src/Shader.cpp
    src/glad.c
    src/imgui.cpp
    src/imgui_demo.cpp
    src/imgui_draw.cpp
    src/imgui_tables.cpp
    src/imgui_widgets.cpp
    src/imgui_impl_glfw.cpp
    src/imgui_impl_opengl3.cpp
)

# 4. Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# 5. Link Directories
# Tells CMake where to find your pre-compiled glfw3.lib
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs)

# 6. Link Libraries
# Combine your executable with OpenGL and GLFW
if(WIN32)
    # On Windows, we link the glfw3.lib you provided in the libs folder
    target_link_libraries(${PROJECT_NAME} PRIVATE glfw3 ${OPENGL_LIBRARIES})
else()
    # Fallback for Mac/Linux just in case
    target_link_libraries(${PROJECT_NAME} PRIVATE glfw ${OPENGL_LIBRARIES})
endif()