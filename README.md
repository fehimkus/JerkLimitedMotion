# Periodic Task Visualization Project

A C++ application that visualizes periodic task execution times using OpenGL with a real-time 2D graph and UI controls.

## Requirements

- C++ compiler with C++11 support or higher  
- CMake 3.22 or higher  
- OpenGL development libraries  
- GLFW3  

---

## Installation

### Debian/Ubuntu

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install build-essential cmake

# Install OpenGL development libraries
sudo apt install libgl1-mesa-dev libglu1-mesa-dev

# Install GLFW development libraries
sudo apt install libglfw3-dev

# Install X11 development libraries (for window/input support)
sudo apt install libx11-dev libxrandr-dev libxi-dev libxinerama-dev libxcursor-dev
```
### Debian/Ubuntu

```bash
# Install build tools, OpenGL, and GLFW
sudo dnf install gcc-c++ cmake mesa-libGL-devel mesa-libGLU-devel glfw-devel libX11-devel libXi-devel libXcursor-devel libXrandr-devel libXinerama-devel
```
### MacOS(Homebrew)

```bash
# Install CMake and GLFW
brew install cmake glfw
```

### Windows(vcpkg)

```bash

# Clone vcpkg if not already installed
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Install dependencies
.\vcpkg install glfw3

# Integrate vcpkg with CMake
.\vcpkg integrate install
```