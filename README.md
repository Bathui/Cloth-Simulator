# Cloth Simulation

## Description
This is a cloth simulation program that uses OpenGL to render a cloth. We can drop the cloth from a certain height and it will fall to the ground. We can also add wind to the cloth and it will blow in the direction of the wind. The other mode is a parachute system where we drop a crate with a parachute attached to it. And we can drop the crate from a certain height and it will fall to the ground with the parachute attached to it. 
## Controls
 - Space: Drop the cloth/crate
 - R: Reset the simulation
 - W: Add wind to the cloth/crate
 - S: Remove wind from the cloth/crate
 - A: Add wind to the left
 - D: Add wind to the right
 - Up Arrow: Add wind up
 - ESC: Exit the program
## Features
    - Cloth simulation
    - Parachute system
    - Wind
    - Collision detection
    - Position-based correction
    - Velocity damping
    - Ground collision
    - Reset simulation
## How to Run

```bash
cmake -B build
cmake --build build
./build/Debug/"Cloth Simulation.exe"
```

## Example Videos

### Cloth Simulation
![clothsimulation](/Result/ClothSimulation.gif)

### Parachute System
![Parachute](/Result/Parachute%20System.gif)
