#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>
#include "profiler.h"
#include "periodic.h"

// Global variables
Profiler profiler;
std::ofstream logFile("motion_log.txt", std::ios::app); // append mode
std::atomic<bool> is_running{false};
std::atomic<double> deltaTime{0.0};
unsigned int cycle_time_us = 100'000; // 100 ms default
double prev_cycle_time = 0.0;
const double tolerance = 0.003;


std::vector<float> time_data, position_data, velocity_data, acceleration_data;
constexpr int MAX_DATA_POINTS = 20000;
float current_time = 0.0f;

// Window constants
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> posDist(-10.0, 10.0);
std::uniform_int_distribution<int> velDist(1, 30);
std::uniform_int_distribution<int> accDist(30, 500);
std::uniform_int_distribution<int> jerkDist(300, 1000);

void write_log(std::string reason)
{

    if (logFile.is_open())
    {
        logFile << "==================== MOTION LOG ====================\n";
        logFile << "Log Reason: " << reason << "\n";
        logFile << "CurrentPosition: " << profiler.CurrentPosition
                << " | CurrentAcceleration: " << profiler.CurrentAcceleration << "\n";
        
        logFile << "Target Position: " << profiler.TargetPosition << "\n";
        logFile << "maxVelocity: " << profiler.MaxVelocity
                << " | maxAcceleration: " << profiler.MaxAcceleration
                << " | maxDeceleration: " << profiler.MaxDeceleration
                << " | Jerk: " << profiler.Jerk << "\n";

        logFile << "-- Time Parameters --\n";
        logFile << "t11: " << profiler.t11 << ", t12: " << profiler.t12 << ", t13: " << profiler.t13 << "\n";
        logFile << "t21: " << profiler.t21 << ", t22: " << profiler.t22 << ", t23: " << profiler.t23 << "\n";

        logFile << "-- Position Distances --\n";
        logFile << "x11: " << profiler.x11 << ", x12: " << profiler.x12 << ", x13: " << profiler.x13 << "\n";
        logFile << "x21: " << profiler.x21 << ", x22: " << profiler.x22 << ", x23: " << profiler.x23 << "\n";

        logFile << "-- Velocity Values --\n";
        logFile << "v11: " << profiler.v11 << ", v12: " << profiler.v12 << ", v13: " << profiler.v13 << "\n";
        logFile << "v21: " << profiler.v21 << ", v22: " << profiler.v22 << ", v23: " << profiler.v23 << "\n";

        logFile << "-- Acceleration Values --\n";
        logFile << "a11: " << profiler.a11 << ", a12: " << profiler.a12 << ", a13: " << profiler.a13 << "\n";
        logFile << "a21: " << profiler.a21 << ", a22: " << profiler.a22 << ", a23: " << profiler.a23 << "\n";

        logFile << "====================================================\n\n";
    }
}

// Function to calculate plot bounds based on input parameters
void calculatePlotBounds(double &pos_min, double &pos_max,
                         double &vel_min, double &vel_max,
                         double &acc_min, double &acc_max)
{
    // Position bounds - based on target position with margin
    pos_min = std::min(0.0, profiler.TargetPosition) - 1.0;
    pos_max = std::max(0.0, profiler.TargetPosition) + 1.0;
    if (pos_min == pos_max)
    {
        pos_min -= 1.0;
        pos_max += 1.0;
    }

    // Velocity bounds - based on max velocity with margin
    vel_min = -profiler.MaxVelocity * 0.1; // Allow small negative for deceleration
    vel_max = profiler.MaxVelocity * 1.1;  // 10% margin

    // Acceleration bounds - based on max acceleration/deceleration with margin
    acc_min = -profiler.MaxDeceleration * 1.1; // 10% margin for deceleration
    acc_max = profiler.MaxAcceleration * 1.1;  // 10% margin for acceleration
}


// ESKİ YAPIDAKİ TRAJECTORY GENERATION MANTIĞI
void *positionGenerator()
{
    double discreteTime{};

    discreteTime = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profiler.begin).count()) * 1e-6f;

    deltaTime = discreteTime - prev_cycle_time;

    switch (profiler.stage)
    {
    case 10: // Accelerate with jerk ------- until max acceleration reached
        if (profiler.CurrentAcceleration >= profiler.a11)
        {
            profiler.CurrentAcceleration = profiler.a11;
            profiler.stage = 20;
        }
        else
        {
            profiler.CurrentAcceleration += profiler.Jerk * deltaTime;
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 20: // Constant acceleration ------- until enough distance for ramp down
        if (profiler.CurrentVelocity >= profiler.v12)
        {
            profiler.stage = 30;
        }
        else
        {
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 30: // Ramp down acceleration ------- until max velocity reached
        if (profiler.CurrentAcceleration <= 0)
        {
            profiler.CurrentAcceleration = 0;
            profiler.stage = 40;
        }
        else
        {
            profiler.CurrentAcceleration -= profiler.Jerk * deltaTime;
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 40: // Cruise
        if ((fabs(profiler.TargetPosition - profiler.CurrentPosition)) <= (profiler.x21 + profiler.x22 + profiler.x23))
        {
            profiler.stage = 50;
        }
        break;

    case 50: // Begin deceleration
        if (fabs(profiler.CurrentAcceleration) > fabs(profiler.a21))
        {
            profiler.CurrentAcceleration = profiler.a21;
            profiler.stage = 60;
        }
        else
        {
            profiler.CurrentAcceleration -= profiler.Jerk * deltaTime;
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 60: // Constant deceleration
        if (profiler.CurrentVelocity <= profiler.v22)
        {
            profiler.stage = 70;
        }
        else
        {
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 70: // End deceleration ramp
    {
        if ((std::fabs(profiler.CurrentPosition) >= std::fabs(profiler.TargetPosition)) ||
            (profiler.CurrentVelocity < 0.0f))
        {
            profiler.CurrentVelocity = 0;
            profiler.CurrentAcceleration = 0;
            profiler.CurrentPosition = profiler.TargetPosition;
            profiler.stage = 0;
        }
        else
        {
            profiler.CurrentAcceleration += profiler.Jerk * deltaTime;
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
    }
    break;

    case 0:
        // Idle state - motion completed
        break;

    default:
        break;
    }

    profiler.CurrentPosition += profiler.CurrentVelocity * deltaTime * profiler.direction;

    prev_cycle_time = discreteTime;
    return nullptr;
}


void startMotion()
{
    is_running = true;

    // ESKİ YAPIDAKİ PROFİL HESAPLAMA
    profiler.TargetDistance = std::fabs(profiler.TargetPosition - profiler.CurrentPosition);
    profiler.direction = (profiler.TargetPosition - profiler.CurrentPosition < 0) ? -1 : 1;
    profiler.CalculatePositionProfile();
    profiler.begin = std::chrono::high_resolution_clock::now();
    profiler.stage = 10;
    prev_cycle_time = 0.0;
    deltaTime = 0.0;
    std::cout << "Motion started - Target: " << profiler.TargetPosition << std::endl;
}

void stopMotion()
{
    is_running = false;
    profiler.stage = 0;
    logFile.close();
    std::cout << "Motion stopped" << std::endl;
}

void resetData()
{
    time_data.clear();
    position_data.clear();
    velocity_data.clear();
    acceleration_data.clear();
    current_time = 0.0f;
    profiler.CurrentPosition = 0.0;
    profiler.CurrentVelocity = 0.0;
    profiler.CurrentAcceleration = 0.0;
    prev_cycle_time = 0.0;
    deltaTime = 0.0;
}


void randomizeParameters()
{
    if (profiler.stage == 0 && is_running)
    {
        usleep(2000000); 
        resetData();
        profiler.TargetPosition = posDist(gen);
        profiler.MaxVelocity = velDist(gen);
        profiler.MaxAcceleration = accDist(gen);
        profiler.MaxDeceleration = accDist(gen);
        profiler.Jerk = jerkDist(gen);


        profiler.TargetDistance = std::fabs(profiler.TargetPosition - profiler.CurrentPosition);
        profiler.direction = (profiler.TargetPosition - profiler.CurrentPosition < 0) ? -1 : 1;
        profiler.CalculatePositionProfile();
        profiler.begin = std::chrono::high_resolution_clock::now();
        profiler.stage = 10;
        prev_cycle_time = 0.0;
        deltaTime = 0.0;
    }


}

// Periodic task - AYRI THREAD'DE ÇALIŞACAK
void my_task()
{
    if (is_running && profiler.stage != 0)
    {
        positionGenerator();

        // Update data for plotting
        current_time = (float)prev_cycle_time;

        time_data.push_back(current_time);
        position_data.push_back((float)profiler.CurrentPosition);
        velocity_data.push_back((float)profiler.CurrentVelocity);
        acceleration_data.push_back((float)profiler.CurrentAcceleration);

        // Keep data size manageable
        if (time_data.size() > MAX_DATA_POINTS)
        {
            time_data.erase(time_data.begin());
            position_data.erase(position_data.begin());
            velocity_data.erase(velocity_data.begin());
            acceleration_data.erase(acceleration_data.begin());
        }
        
        if (profiler.stage == 40)
        {
            if (profiler.CurrentVelocity > profiler.MaxVelocity)
            {
                write_log("Current velocity exceeded max velocity during cruise phase.");
            }
        }
        else if (profiler.stage >= 10 && profiler.stage < 40)
        {
            if (std::fabs(profiler.CurrentAcceleration) > std::fabs(profiler.MaxAcceleration))
            {
                write_log("Current acceleration exceeded max acceleration/deceleration during motion.");
            }
        }
        else if (profiler.stage > 40)
        {
            if (std::fabs(profiler.CurrentAcceleration) > std::fabs(profiler.MaxDeceleration))
            {
                write_log("Current deceleration exceeded max deceleration during motion.");
            }
        }
        else
        {
            if(fabs(profiler.CurrentPosition) > (profiler.TargetDistance))
            {
                write_log("Current position exceeded target position during motion.");
            }
        }
        
    }
}



// Function to draw reference lines using ImPlot's native functions
void drawReferenceLine(double value, const ImVec4 &color)
{
    double x_min = ImPlot::GetPlotLimits().X.Min;
    double x_max = ImPlot::GetPlotLimits().X.Max;

    float y_value = (float)value;
    float x_values[2] = {(float)x_min, (float)x_max};
    float y_values[2] = {y_value, y_value};

    ImPlot::SetNextLineStyle(color, 2.0f);
    ImPlot::PlotLine("##ref", x_values, y_values, 2);
}

int main()
{
    // Profiler başlangıç değerleri
    profiler.TargetPosition = 2.526;
    profiler.MaxVelocity = 1.000;
    profiler.MaxAcceleration = 339.0;
    profiler.MaxDeceleration = 373.0;
    profiler.Jerk = 773.0;

    profiler.CurrentPosition = 0.0;
    profiler.CurrentVelocity = 0.0;
    profiler.CurrentAcceleration = 0.0;
    profiler.stage = 0;
    static std::thread rnd_thread;
    static std::atomic<bool> rnd_thread_active{false};
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Daha uyumlu OpenGL sürümü
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // macOS için

    // Create window with fixed size
    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
                                          "Motion Control System", NULL, NULL);

    // Pencere oluşturulamazsa daha düşük OpenGL sürümü dene
    if (!window)
    {
        std::cout << "Failed to create window with OpenGL 3.2, trying compatible mode..." << std::endl;

        // Reset window hints
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

        window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
                                  "Motion Control System", NULL, NULL);
    }

    if (!window)
    {
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
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    // Setup style
    ImGui::StyleColorsDark();

    // GLSL version detection
    const char *glsl_version = "#version 150";
    if (glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR) == 2)
    {
        glsl_version = "#version 120";
    }

    std::cout << "Using GLSL version: " << glsl_version << std::endl;

    // Setup Platform/Renderer backends
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
    {
        std::cerr << "Failed to initialize ImGui GLFW backend" << std::endl;
        return -1;
    }

    if (!ImGui_ImplOpenGL3_Init(glsl_version))
    {
        std::cerr << "Failed to initialize ImGui OpenGL3 backend" << std::endl;
        return -1;
    }

    // AYRI THREAD'DE PERIODIC TASK BAŞLAT
    std::thread periodic_thread([&]()
                                {
        update_period(cycle_time_us);
        run_periodic(my_task); });
    periodic_thread.detach();

    std::cout << "Application started successfully!" << std::endl;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll events
        glfwPollEvents();

        // Start Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Calculate plot bounds based on current input parameters
        double pos_min, pos_max, vel_min, vel_max, acc_min, acc_max;
        calculatePlotBounds(pos_min, pos_max, vel_min, vel_max, acc_min, acc_max);

        // Main control window - left side (YENİ YAPIDAKİ MODERN ARAYÜZ)
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(400, WINDOW_HEIGHT));
        if (ImGui::Begin("Control Panel", nullptr,
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse))
        {

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
            if (ImGui::Button("START", ImVec2(120, 40)))
            {
                startMotion();
            }
            ImGui::SameLine();

            if (ImGui::Button("STOP", ImVec2(120, 40)))
            {
                rnd_thread_active = false;
                stopMotion();
            }

            ImGui::Spacing();
            if (ImGui::Button("RANDOMIZE", ImVec2(120, 40)))
            {
                // Clear and reopen logfile before starting randomization
                logFile.close();
                logFile.open("motion_log.txt", std::ios::trunc);
                logFile.close();
                logFile.open("motion_log.txt", std::ios::app);
                is_running = true;

                // Toggle background randomizer thread
                if (!rnd_thread_active.load())
                {
                    rnd_thread_active = true;
                    rnd_thread = std::thread([&]()
                                             {
                                                 while (rnd_thread_active.load())
                                                 {
                                                     randomizeParameters();
                                                     std::this_thread::sleep_for(std::chrono::milliseconds(500)); // adjust interval as needed
                                                 }
                                             });
                    rnd_thread.detach();
                }
                else
                {
                    // Stop the background thread loop
                    rnd_thread_active = false;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("RESET DATA", ImVec2(120, 40)))
            {
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
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Position time Parameters:");

            // Display t11..t23 side-by-side
            ImGui::Text("t11: %.3f", profiler.t11); ImGui::SameLine(0, 20);
            ImGui::Text("t12: %.3f", profiler.t12); ImGui::SameLine(0, 20);
            ImGui::Text("t13: %.3f", profiler.t13);
            ImGui::Text("t21: %.3f", profiler.t21); ImGui::SameLine(0, 20);
            ImGui::Text("t22: %.3f", profiler.t22); ImGui::SameLine(0, 20);
            ImGui::Text("t23: %.3f", profiler.t23);

            ImGui::Text("Position distance Parameters:");
            ImGui::Text("x11: %.3f", profiler.x11); ImGui::SameLine(0, 20);
            ImGui::Text("x12: %.3f", profiler.x12); ImGui::SameLine(0, 20);
            ImGui::Text("x13: %.3f", profiler.x13);
            ImGui::Text("x21: %.3f", profiler.x21); ImGui::SameLine(0, 20);
            ImGui::Text("x22: %.3f", profiler.x22); ImGui::SameLine(0, 20);
            ImGui::Text("x23: %.3f", profiler.x23);
            ImGui::Text("itogo: %.3f", profiler.x11 + profiler.x12 + profiler.x13 + profiler.x21 + profiler.x22 + profiler.x23);

            ImGui::Text("Velocity Parameters:");
            ImGui::Text("v11: %.3f", profiler.v11); ImGui::SameLine(0, 20);
            ImGui::Text("v12: %.3f", profiler.v12); ImGui::SameLine(0, 20);
            ImGui::Text("v13: %.3f", profiler.v13);
            ImGui::Text("v21: %.3f", profiler.v21); ImGui::SameLine(0, 20);
            ImGui::Text("v22: %.3f", profiler.v22); ImGui::SameLine(0, 20);
            ImGui::Text("v23: %.3f", profiler.v23);

            ImGui::Text("Acceleration Parameters:");
            ImGui::Text("a11: %.3f", profiler.a11); ImGui::SameLine(0, 20);
            ImGui::Text("a12: %.3f", profiler.a12); ImGui::SameLine(0, 20);
            ImGui::Text("a13: %.3f", profiler.a13);
            ImGui::Text("a21: %.3f", profiler.a21); ImGui::SameLine(0, 20);
            ImGui::Text("a22: %.3f", profiler.a22); ImGui::SameLine(0, 20);
            ImGui::Text("a23: %.3f", profiler.a23);

            ImGui::End();
        }

        // Plot window - right side (YENİ YAPIDAKİ MODERN GRAFİKLER)
        ImGui::SetNextWindowPos(ImVec2(400, 0));
        ImGui::SetNextWindowSize(ImVec2(WINDOW_WIDTH - 400, WINDOW_HEIGHT));
        if (ImGui::Begin("Motion Plots", nullptr,
                         ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse))
        {

            // Calculate time window for X-axis (show last 10 seconds or all data)
            float time_min = 0.0f;
            float time_max = current_time;
            if (current_time > 10.0f)
            {
                time_min = current_time - 10.0f; // Show last 10 seconds
            }

            // Position plot
            if (ImPlot::BeginPlot("Position", ImVec2(-1, 250)))
            {
                ImPlot::SetupAxes("Time (s)", "Position");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, pos_min, pos_max, ImGuiCond_Always);
                if (!time_data.empty() && !position_data.empty())
                {
                    ImPlot::PlotLine("", time_data.data(), position_data.data(), time_data.size());
                }
                // Add reference line for target position
                drawReferenceLine(profiler.TargetPosition, ImVec4(1, 0, 0, 0.5f));
                ImPlot::EndPlot();
            }

            // Velocity plot
            if (ImPlot::BeginPlot("Velocity", ImVec2(-1, 250)))
            {
                ImPlot::SetupAxes("Time (s)", "Velocity");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, vel_min, vel_max, ImGuiCond_Always);
                if (!time_data.empty() && !velocity_data.empty())
                {
                    ImPlot::PlotLine("", time_data.data(), velocity_data.data(), time_data.size());
                }
                // Add reference line for max velocity
                drawReferenceLine(profiler.MaxVelocity, ImVec4(1, 0, 0, 0.5f));
                ImPlot::EndPlot();
            }

            // Acceleration plot
            if (ImPlot::BeginPlot("Acceleration", ImVec2(-1, 250)))
            {
                ImPlot::SetupAxes("Time (s)", "Acceleration");
                ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, acc_min, acc_max, ImGuiCond_Always);
                if (!time_data.empty() && !acceleration_data.empty())
                {
                    ImPlot::PlotLine("", time_data.data(), acceleration_data.data(), time_data.size());
                }
                // Add reference lines for max acceleration and deceleration
                drawReferenceLine(profiler.MaxAcceleration, ImVec4(1, 0, 0, 0.5f));
                drawReferenceLine(-profiler.MaxDeceleration, ImVec4(0, 1, 0, 0.5f));
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