#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <deque>
#include "periodic.h"
#include "profiler.h"

// GLFW and ImGui headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "implot.h"
#include <GLFW/glfw3.h>


// Global objects
Profiler profiler;
std::atomic<bool> is_running{false};
std::atomic<double> deltaTime{0.0};
unsigned int cycle_time_us = 100'000; // 100 ms default
unsigned int BUFFER_SIZE = 2'000'000'000 / cycle_time_us;
double prev_cycle_time = 0.0;
// --------------------------
// UI BASE DIMENSIONS
// --------------------------
constexpr float BASE_WIDTH = 1280.0f;
constexpr float BASE_HEIGHT = 1280.0f;

// Helper to calculate scaling factors
float getScaleFactorX(float currentWidth)
{
    return currentWidth / BASE_WIDTH;
}

float getScaleFactorY(float currentHeight)
{
    return currentHeight / BASE_HEIGHT;
}

// Random number generators
std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> posDist(-10.0, 10.0);
std::uniform_int_distribution<int> velDist(5, 30);
std::uniform_int_distribution<int> accDist(30, 100);

// User input values
struct UserInput
{
    double target_pos = 5.0;
    double max_vel = 10.0;
    double max_acc = 70.0;
    double max_dec = 40.0;
    double jerk = 800.0;
} user_input;

// --------------------------
// Circular buffer for plotting
// --------------------------
struct PlotBuffer
{
    std::vector<float> x_data;
    std::vector<float> y_data;
    size_t index = 0;
    bool full = false;
    float latest_time = 0.0f;

    PlotBuffer()
    {
        x_data.resize(BUFFER_SIZE, 0.0f);
        y_data.resize(BUFFER_SIZE, 0.0f);
    }

    void addPoint(float x, float y)
    {
        x_data[index] = x;
        y_data[index] = y;
        index = (index + 1) % BUFFER_SIZE;
        if (index == 0)
            full = true;
        latest_time = x;
    }

    float earliestTime() const
    {
        return latest_time - (BUFFER_SIZE * deltaTime);
    }

    std::pair<float, float> getYBounds() const
    {
        float minVal = 1e9f;
        float maxVal = -1e9f;
        size_t count = full ? BUFFER_SIZE : index;
        for (size_t i = 0; i < count; i++)
        {
            if (y_data[i] < minVal)
                minVal = y_data[i];
            if (y_data[i] > maxVal)
                maxVal = y_data[i];
        }
        if (count == 0)
        {
            minVal = -1.0f;
            maxVal = 1.0f;
        }
        return {minVal, maxVal};
    }
};

// --------------------------
// Global buffers for three plots
// --------------------------
PlotBuffer positionPlot;
PlotBuffer velocityPlot;
PlotBuffer accelerationPlot;

void updateProfileFromUserInput()
{
    profiler.TargetPosition = user_input.target_pos;
    profiler.MaxVelocity = user_input.max_vel;
    profiler.MaxAcceleration = user_input.max_acc;
    profiler.MaxDeceleration = user_input.max_dec;
    profiler.Jerk = user_input.jerk;

    profiler.TargetDistance = std::fabs(profiler.TargetPosition - profiler.CurrentPosition);
    profiler.direction = (profiler.TargetPosition - profiler.CurrentPosition < 0) ? -1 : 1;

    profiler.CalculatePositionProfile();
    profiler.begin = std::chrono::high_resolution_clock::now();
    profiler.stage = 10;

    std::cout << "Profile updated - Target: " << profiler.TargetPosition
              << ", MaxVel: " << profiler.MaxVelocity
              << ", Stage: " << profiler.stage << std::endl;
}

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
            // std::cout << "Stage 10: Max acceleration reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // profiler.CurrentVelocity = profiler.v12;
            // std::cout << "Stage 20: Max velocity reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // std::cout << "Stage 30: acceleration need to decrease" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // std::cout << "Stage 40: Deceleration needed" << std::endl;
            // std::cout << "Target position: " << profiler.TargetPosition << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
            profiler.stage = 50;
        }
        break;

    case 50: // Begin deceleration
        if (fabs(profiler.CurrentAcceleration) > fabs(profiler.a21))
        {
            profiler.CurrentAcceleration = profiler.a21;
            // std::cout << "Stage 50: Max deceleration reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // profiler.CurrentVelocity = profiler.v22;
            // std::cout << "Stage 60: Min velocity reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // std::cout << "Stage 70: Target position reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
            profiler.CurrentVelocity = 0;
            profiler.CurrentAcceleration = 0;
            profiler.CurrentPosition = profiler.TargetPosition;
            profiler.stage = 0;
        }
        else
        {
            // std::cout << "pos " << profiler.CurrentPosition << " vel " << profiler.CurrentVelocity << " acc " << profiler.CurrentAcceleration << " stage " << profiler.stage << std::endl;
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

// Periodic task
void my_task()
{

    if (is_running && profiler.stage != 0)
    {
        positionGenerator();
        // std::cout << "pos " << profiler.CurrentPosition << " vel " << profiler.CurrentVelocity << " acc " << profiler.CurrentAcceleration << " stage " << profiler.stage << std::endl;
        positionPlot.addPoint(static_cast<float>(prev_cycle_time), static_cast<float>(profiler.CurrentPosition));
        velocityPlot.addPoint(static_cast<float>(prev_cycle_time), static_cast<float>(profiler.CurrentVelocity));
        accelerationPlot.addPoint(static_cast<float>(prev_cycle_time), static_cast<float>(profiler.CurrentAcceleration));
    }

}

void start()
{


    // Reset profiler state
    profiler.CurrentPosition = 0;
    profiler.CurrentVelocity = 0;
    profiler.CurrentAcceleration = 0;


}

void randomize()
{
    user_input.target_pos = posDist(rng);
    user_input.max_vel = velDist(rng);
    user_input.max_acc = accDist(rng);
    user_input.max_dec = accDist(rng);
    user_input.jerk = ((user_input.max_acc + user_input.max_dec) / 2.0) * 12.5;


}

void stop()
{

    // Clear graph data when stopping

    // Reset profiler to idle state
    profiler.stage = 0;
    profiler.CurrentVelocity = 0;
    profiler.CurrentAcceleration = 0;

    std::cout << "Motion STOPPED - Graph cleared" << std::endl;
}



// --------------------------
// Utility: Draw a sliding plot
// --------------------------
void drawPlot(const char *title, PlotBuffer &buffer, float scaleY)
{
    if (ImPlot::BeginPlot(title, ImVec2(-1, 0)))
    {
        // Calculate dynamic Y bounds
        auto [minY, maxY] = buffer.getYBounds();

        // Add some padding for clarity
        float padding = (maxY - minY) * 0.1f;
        if (padding < 0.001f)
            padding = 0.1f;
        minY -= padding;
        maxY += padding;

        // Setup axes dynamically
        ImPlot::SetupAxes("Time", "Value");
        ImPlot::SetupAxisLimits(ImAxis_X1, buffer.earliestTime(), buffer.latest_time, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, minY, maxY, ImGuiCond_Always);

        // Draw circular buffer
        if (buffer.full)
        {
            ImPlot::PlotLine("Signal",
                             &buffer.x_data[buffer.index], &buffer.y_data[buffer.index],
                             BUFFER_SIZE - buffer.index);
            if (buffer.index > 0)
            {
                ImPlot::PlotLine("Signal",
                                 buffer.x_data.data(), buffer.y_data.data(),
                                 buffer.index);
            }
        }
        else
        {
            ImPlot::PlotLine("Signal",
                             buffer.x_data.data(), buffer.y_data.data(),
                             static_cast<int>(buffer.index));
        }

        ImPlot::EndPlot();
    }
}

// --------------------------
// GLFW error callback
// --------------------------
void glfw_error_callback(int error, const char *description)
{
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}



// --------------------------
// Main
// --------------------------
int main()
{
    // Setup GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        return -1;
    }

    // Create window using base dimensions
    GLFWwindow *window = glfwCreateWindow((int)BASE_WIDTH, (int)BASE_HEIGHT,
                                          "ImGui + ImPlot - Scaled Multi Plot Example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();


    std::thread([=]()
                {
        update_period(cycle_time_us);
        run_periodic(my_task); })
        .detach();


    // Main Loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Get window size to compute scale factors
        int windowWidth, windowHeight;
        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        float scaleX = getScaleFactorX((float)windowWidth);
        float scaleY = getScaleFactorY((float)windowHeight);

        // Scale fonts dynamically
        io.FontGlobalScale = std::min(scaleX, scaleY);

        // Start new ImGui frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Motion Parameters");

        ImGui::InputDouble("Target Position", &user_input.target_pos, 0.1, 1.0, "%.3f");
        ImGui::InputDouble("Max Velocity", &user_input.max_vel, 0.1, 1.0, "%.3f");
        ImGui::InputDouble("Max Accel", &user_input.max_acc, 0.1, 1.0, "%.3f");
        ImGui::InputDouble("Max Decel", &user_input.max_dec, 0.1, 1.0, "%.3f");
        ImGui::InputDouble("Jerk", &user_input.jerk, 1.0, 10.0, "%.3f");


        ImGui::End();

        ImGui::Begin("Main Window");
        // --------------------------
        // Control Panel
        // --------------------------
        if (ImGui::Button("Restart", ImVec2(100 * scaleX, 40 * scaleY)))
        {
            // Stop current run
            is_running = false;

            // Reset profiler state
            profiler.CurrentPosition = 0.0;
            profiler.CurrentVelocity = 0.0;
            profiler.CurrentAcceleration = 0.0;

            // Clear all graphs
            positionPlot = PlotBuffer();
            velocityPlot = PlotBuffer();
            accelerationPlot = PlotBuffer();
            deltaTime = 0.0;
            prev_cycle_time = 0.0;
            // Apply parameters
            profiler.TargetPosition = user_input.target_pos;
            profiler.MaxVelocity = user_input.max_vel;
            profiler.MaxAcceleration = user_input.max_acc;
            profiler.MaxDeceleration = user_input.max_dec;
            profiler.Jerk = user_input.jerk;
            profiler.TargetDistance = std::fabs(profiler.TargetPosition - profiler.CurrentPosition);
            profiler.direction = (profiler.TargetPosition - profiler.CurrentPosition < 0) ? -1 : 1;
            profiler.CalculatePositionProfile();
            profiler.begin = std::chrono::high_resolution_clock::now();
            profiler.stage = 10;

            // Start motion again
            is_running = true;

            std::cout << "Motion RESTARTED with cleared graphs and parameters:" << std::endl;
            std::cout << "  Target Position: " << user_input.target_pos << std::endl;
            std::cout << "  Max Velocity: " << user_input.max_vel << std::endl;
            std::cout << "  Max Acceleration: " << user_input.max_acc << std::endl;
            std::cout << "  Max Deceleration: " << user_input.max_dec << std::endl;
            std::cout << "  Jerk: " << user_input.jerk << std::endl;
        }

        ImGui::SameLine();
        if (ImGui::Button("Stop", ImVec2(100 * scaleX, 40 * scaleY)))
        {
            is_running = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Randomize", ImVec2(120 * scaleX, 40 * scaleY)))
        {
            randomize();
        }


        ImGui::Separator();

        // --------------------------
        // Dynamic space allocation for 3 plots
        // --------------------------
        ImVec2 available = ImGui::GetContentRegionAvail();
        float total_height = available.y;
        float plot_height = (total_height) / 3.50f;

        // Draw Position plot
        ImGui::Text("Position");
        ImGui::BeginChild("PositionPlot", ImVec2(available.x, plot_height), true);
        drawPlot("Position", positionPlot, scaleY);
        ImGui::EndChild();

        ImGui::Spacing();

        // Draw Velocity plot
        ImGui::Text("Velocity");
        ImGui::BeginChild("VelocityPlot", ImVec2(available.x, plot_height), true);
        drawPlot("Velocity", velocityPlot, scaleY);
        ImGui::EndChild();

        ImGui::Spacing();

        // Draw Acceleration plot
        ImGui::Text("Acceleration");
        ImGui::BeginChild("AccelerationPlot", ImVec2(available.x, plot_height), true);
        drawPlot("Acceleration", accelerationPlot, scaleY);
        ImGui::EndChild();

        ImGui::End();


        // --------------------------
        // Rendering
        // --------------------------
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}