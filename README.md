# Periodic Task Visualization Project

A C++ application that visualizes periodic task execution times using OpenGL with a real-time 2D graph and UI controls.

## Requirements

- C++ compiler with C++11 support
- CMake 3.22 or higher
- OpenGL development libraries
- GLFW3
- GLUT/FreeGLUT

## Installation for Debian/Ubuntu Based Systems

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install build-essential cmake

# Install OpenGL and GLUT development libraries
sudo apt install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev

# Install GLFW development libraries
sudo apt install libglfw3-dev

# Install X11 development libraries (if not already installed)
sudo apt install libx11-dev libxrandr-dev libxi-dev libxinerama-dev libxcursor-dev