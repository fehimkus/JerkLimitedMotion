cmake_minimum_required(VERSION 3.22)

project(posgen LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_TESTS OFF)

# ---- Find OpenGL and GLUT ----
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# ---- Executable ----
add_executable(${PROJECT_NAME}
    posgen.cpp
    periodic.cpp
    profiler.cpp
    graph.cpp
)

# ---- Link libraries ----
target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenGL::GL
    GLUT::GLUT
)
