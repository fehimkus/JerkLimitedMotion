#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <algorithm>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

// Simple profiler structure
struct Profiler {
    double CurrentPosition = 0.0;
    double CurrentVelocity = 0.0;
    double CurrentAcceleration = 0.0;
    double TargetPosition = 5.0;
    double MaxVelocity = 10.0;
    double MaxAcceleration = 70.0;
    double MaxDeceleration = 40.0;
    double Jerk = 800.0;
    int stage = 0;
};

// Global variables
Profiler profiler;
std::atomic<bool> is_running{false};
std::vector<float> time_data, position_data, velocity_data, acceleration_data;
constexpr int MAX_DATA_POINTS = 2000;
float current_time = 0.0f;

// Window constants
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());

// Function to calculate plot bounds based on input parameters
void calculatePlotBounds(double& pos_min, double& pos_max, 
                        double& vel_min, double& vel_max,
                        double& acc_min, double& acc_max) {
    // Position bounds - based on target position with margin
    pos_min = std::min(0.0, profiler.TargetPosition) - 1.0;
    pos_max = std::max(0.0, profiler.TargetPosition) + 1.0;
    if (pos_min == pos_max) {
        pos_min -= 1.0;
        pos_max += 1.0;
    }

    // Velocity bounds - based on max velocity with margin
    vel_min = -profiler.MaxVelocity * 0.1; // Allow small negative for deceleration
    vel_max = profiler.MaxVelocity * 1.1;   // 10% margin

    // Acceleration bounds - based on max acceleration/deceleration with margin
    acc_min = -profiler.MaxDeceleration * 1.1; // 10% margin for deceleration
    acc_max = profiler.MaxAcceleration * 1.1;   // 10% margin for acceleration
}

void updateMotion(float dt) {
    if (!is_running || profiler.stage == 0) return;

    // Simplified motion logic
    switch (profiler.stage) {
        case 1: // Acceleration phase
            profiler.CurrentAcceleration = std::min(profiler.CurrentAcceleration + profiler.Jerk * dt, profiler.MaxAcceleration);
            profiler.CurrentVelocity = std::min(profiler.CurrentVelocity + profiler.CurrentAcceleration * dt, profiler.MaxVelocity);
            if (profiler.CurrentVelocity >= profiler.MaxVelocity) {
                profiler.stage = 2;
            }
            break;
        case 2: // Constant velocity
            if (std::abs(profiler.CurrentPosition - profiler.TargetPosition) < 10.0) {
                profiler.stage = 3;
            }
            break;
        case 3: // Deceleration phase
            profiler.CurrentAcceleration = std::max(profiler.CurrentAcceleration - profiler.Jerk * dt, -profiler.MaxDeceleration);
            profiler.CurrentVelocity = std::max(profiler.CurrentVelocity + profiler.CurrentAcceleration * dt, 0.0);
            if (profiler.CurrentVelocity <= 0.0) {
                profiler.stage = 0;
                profiler.CurrentPosition = profiler.TargetPosition;
                is_running = false;
            }
            break;
    }

    profiler.CurrentPosition += profiler.CurrentVelocity * dt;
    
    // Update data for plotting
    current_time += dt;
    
    time_data.push_back(current_time);
    position_data.push_back((float)profiler.CurrentPosition);
    velocity_data.push_back((float)profiler.CurrentVelocity);
    acceleration_data.push_back((float)profiler.CurrentAcceleration);
    
    // Keep data size manageable
    if (time_data.size() > MAX_DATA_POINTS) {
        time_data.erase(time_data.begin());
        position_data.erase(position_data.begin());
        velocity_data.erase(velocity_data.begin());
        acceleration_data.erase(acceleration_data.begin());
    }
}

void startMotion() {
    is_running = true;
    profiler.stage = 1;
    std::cout << "Motion started - Target: " << profiler.TargetPosition << std::endl;
}

void stopMotion() {
    is_running = false;
    profiler.stage = 0;
    std::cout << "Motion stopped" << std::endl;
}

void randomizeParameters() {
    std::uniform_real_distribution<double> pos_dist(-10.0, 10.0);
    std::uniform_real_distribution<double> vel_dist(5.0, 30.0);
    std::uniform_real_distribution<double> acc_dist(30.0, 100.0);
    
    profiler.TargetPosition = pos_dist(gen);
    profiler.MaxVelocity = vel_dist(gen);
    profiler.MaxAcceleration = acc_dist(gen);
    profiler.MaxDeceleration = acc_dist(gen);
    profiler.Jerk = (profiler.MaxAcceleration + profiler.MaxDeceleration) * 10.0;
    
    std::cout << "Parameters randomized" << std::endl;
}

void resetData() {
    time_data.clear();
    position_data.clear();
    velocity_data.clear();
    acceleration_data.clear();
    current_time = 0.0f;
    profiler.CurrentPosition = 0.0;
    profiler.CurrentVelocity = 0.0;
    profiler.CurrentAcceleration = 0.0;
}

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Daha uyumlu OpenGL sürümü
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // macOS için

    // Create window with fixed size
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, 
                                         "Motion Control System", NULL, NULL);
    
    // Pencere oluşturulamazsa daha düşük OpenGL sürümü dene
    if (!window) {
        std::cout << "Failed to create window with OpenGL 3.2, trying compatible mode..." << std::endl;
        
        // Reset window hints
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        
        window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, 
                                 "Motion Control System", NULL, NULL);
    }

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    
    // Setup style
    ImGui::StyleColorsDark();

    // GLSL version detection
    const char* glsl_version = "#version 150";
    if (glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR) == 2) {
        glsl_version = "#version 120";
    }
    
    std::cout << "Using GLSL version: " << glsl_version << std::endl;

    // Setup Platform/Renderer backends
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend" << std::endl;
        return -1;
    }
    
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        std::cerr << "Failed to initialize ImGui OpenGL3 backend" << std::endl;
        return -1;
    }

    // Variables for timing
    auto last_time = std::chrono::high_resolution_clock::now();

    std::cout << "Application started successfully!" << std::endl;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Calculate delta time
        auto current_time_point = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(current_time_point - last_time).count();
        last_time = current_time_point;

        // Poll events
        glfwPollEvents();

        // Start Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Update motion physics
        updateMotion(dt);

        // Calculate plot bounds based on current input parameters
        double pos_min, pos_max, vel_min, vel_max, acc_min, acc_max;
        calculatePlotBounds(pos_min, pos_max, vel_min, vel_max, acc_min, acc_max);

        // Main control window - left side
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(400, WINDOW_HEIGHT));
        if (ImGui::Begin("Control Panel", nullptr, 
                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | 
                    ImGuiWindowFlags_NoCollapse)) {

            ImGui::Text("Motion Parameters");
            ImGui::Separator();
            
            // Input fields
            ImGui::InputDouble("Target Position", &profiler.TargetPosition, 0.1, 1.0, "%.3f");
            ImGui::InputDouble("Max Velocity", &profiler.MaxVelocity, 0.1, 1.0, "%.3f");
            ImGui::InputDouble("Max Acceleration", &profiler.MaxAcceleration, 1.0, 10.0, "%.3f");
            ImGui::InputDouble("Max Deceleration", &profiler.MaxDeceleration, 1.0, 10.0, "%.3f");
            ImGui::InputDouble("Jerk", &profiler.Jerk, 10.0, 100.0, "%.3f");

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Control buttons
            if (ImGui::Button("START", ImVec2(120, 40))) {
                startMotion();
            }
            ImGui::SameLine();
            if (ImGui::Button("STOP", ImVec2(120, 40))) {
                stopMotion();
            }
            
            ImGui::Spacing();
            
            if (ImGui::Button("RANDOMIZE", ImVec2(120, 40))) {
                randomizeParameters();
            }
            ImGui::SameLine();
            if (ImGui::Button("RESET DATA", ImVec2(120, 40))) {
                resetData();
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // Status information
            ImGui::Text("Status: %s", is_running ? "RUNNING" : "STOPPED");
            ImGui::Text("Stage: %d", profiler.stage);
            ImGui::Text("Position: %.3f", profiler.CurrentPosition);
            ImGui::Text("Velocity: %.3f", profiler.CurrentVelocity);
            ImGui::Text("Acceleration: %.3f", profiler.CurrentAcceleration);

            // Show current plot bounds
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Plot Ranges:");
            ImGui::Text("Position: [%.1f, %.1f]", pos_min, pos_max);
            ImGui::Text("Velocity: [%.1f, %.1f]", vel_min, vel_max);
            ImGui::Text("Acceleration: [%.1f, %.1f]", acc_min, acc_max);

            ImGui::End();
        }

        // Plot window - right side
        ImGui::SetNextWindowPos(ImVec2(400, 0));
        ImGui::SetNextWindowSize(ImVec2(WINDOW_WIDTH - 400, WINDOW_HEIGHT));
        if (ImGui::Begin("Motion Plots", nullptr, 
                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | 
                    ImGuiWindowFlags_NoCollapse)) {

            // Calculate time window for X-axis (show last 10 seconds or all data)
            float time_min = 0.0f;
            float time_max = current_time;
            if (current_time > 10.0f) {
                time_min = current_time - 10.0f; // Show last 10 seconds
            }

            // Position plot
            if (ImPlot::BeginPlot("Position", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (s)", "Position");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, pos_min, pos_max, ImGuiCond_Always);
                if (!time_data.empty() && !position_data.empty()) {
                    ImPlot::PlotLine("Position", time_data.data(), position_data.data(), time_data.size());
                }
                // Add reference lines
                ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f), 2.0f);
                ImPlot::PlotLine("Target", &time_min, &profiler.TargetPosition, 2, 0);
                ImPlot::EndPlot();
            }

            // Velocity plot
            if (ImPlot::BeginPlot("Velocity", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (s)", "Velocity");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, vel_min, vel_max, ImGuiCond_Always);
                if (!time_data.empty() && !velocity_data.empty()) {
                    ImPlot::PlotLine("Velocity", time_data.data(), velocity_data.data(), time_data.size());
                }
                // Add reference lines
                ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f), 2.0f);
                ImPlot::PlotLine("Max Vel", &time_min, &profiler.MaxVelocity, 2, 0);
                ImPlot::EndPlot();
            }

            // Acceleration plot
            if (ImPlot::BeginPlot("Acceleration", ImVec2(-1, 250))) {
                ImPlot::SetupAxes("Time (s)", "Acceleration");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, acc_min, acc_max, ImGuiCond_Always);
                if (!time_data.empty() && !acceleration_data.empty()) {
                    ImPlot::PlotLine("Acceleration", time_data.data(), acceleration_data.data(), time_data.size());
                }
                // Add reference lines
                ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f), 2.0f);
                ImPlot::PlotLine("Max Acc", &time_min, &profiler.MaxAcceleration, 2, 0);
                ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f), 2.0f);
                ImPlot::PlotLine("Max Dec", &time_min, &profiler.MaxDeceleration, 2, 0);
                ImPlot::EndPlot();
            }

            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}