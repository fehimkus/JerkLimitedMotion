# Periodic Task / Motion Visualization (ImGui + ImPlot)

This app shows real-time **Position / Velocity / Acceleration** plots using **Dear ImGui** and **ImPlot**.
It also includes a simple jerk-limited simulator to **Calculate** a motion profile from inputs.

## What’s inside
- **GLFW** window + OpenGL2 backend
- **Dear ImGui** UI (controls + windows)
- **ImPlot** for fast plotting (3 stacked subplots)
- Random motion generator (Start/Stop)
- “Calculate” button: simulate trajectory with target, Vmax, Amax, Dmax, Jmax

## Build Requirements
- CMake ≥ 3.22
- A C++17 compiler
- OpenGL dev headers
- X11 dev packages on Linux (for GLFW)

### Debian/Ubuntu
```bash
sudo apt update
sudo apt install -y build-essential cmake \
  libgl1-mesa-dev libglu1-mesa-dev \
  libx11-dev libxrandr-dev libxi-dev libxinerama-dev libxcursor-dev
