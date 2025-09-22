cmake_minimum_required(VERSION 3.22)
project(posgen LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_TESTS OFF)

# ---- Find OpenGL ----
find_package(OpenGL REQUIRED)

# ---- Find GLFW ----
find_package(glfw3 REQUIRED)

# ---- Executable ----
add_executable(${PROJECT_NAME}
    posgen.cpp
    periodic.cpp
    profiler.cpp
    graph.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenGL::GL
    glfw
)

if(APPLE)
    # GLFW needs these macOS frameworks
    target_link_libraries(${PROJECT_NAME} PRIVATE
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreFoundation"
        "-framework CoreVideo"
    )
endif()
