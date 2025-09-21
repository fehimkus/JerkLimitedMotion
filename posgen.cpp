#include <random>
#include <thread>
#include "graph.h"
#include "periodic.h"
#include "profiler.h"

#define BufferSize 10000

Graph graph;
Profiler profiler;
double prev_cycle_time = 0.0;
static auto start_time = std::chrono::high_resolution_clock::now();

std::mt19937 rng(std::random_device{}()); // random engine
std::uniform_real_distribution<double> posDist(-10.0, 10.0);
std::uniform_int_distribution<int> velDist(5, 30);
std::uniform_int_distribution<int> accDist(30, 100);

void *positionGenerator()
{

    double discreteTime{};

    discreteTime = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>
        (std::chrono::system_clock::now() - profiler.begin).count()) * 1e-6f;

    double deltaTime{};
    deltaTime = discreteTime - prev_cycle_time;

    switch (profiler.stage)
    {
    case 10: // Accelerate with jerk ------- until max acceleration reached
        if (profiler.CurrentAcceleration >= profiler.a11)
        {
            // std::cout << "Stage 10: Max acceleration reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // std::cout << "Stage 20: Max velocity reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
            profiler.CurrentVelocity = profiler.v12;
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
            // std::cout << "Stage 30: acceleration need to decrease" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
        if ((abs(profiler.TargetPosition - profiler.CurrentPosition)) <= (profiler.x21 + profiler.x22 + profiler.x23))
        {
            // std::cout << "Stage 40: Deceleration needed" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
            profiler.stage = 50;
        }
            break;

    case 50: // Begin deceleration
        if (abs(profiler.CurrentAcceleration) > abs(profiler.a21))
        {
            // std::cout << "Stage 50: Max deceleration reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
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
            // std::cout << "Stage 60: Min velocity reached" << std::endl;
            // std::cout << "CurrentAcceleration: " << profiler.CurrentAcceleration << " \n";
            // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
            // std::cout << "CurrentPosition: " << profiler.CurrentPosition << " \n";
            profiler.CurrentVelocity = profiler.v22;
            profiler.stage = 70;
        }
        else
        {
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
        break;

    case 70: // End deceleration ramp
    {
        if ((std::abs(profiler.CurrentPosition) >= std::abs(profiler.TargetPosition)) ||
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
            profiler.CurrentAcceleration += profiler.Jerk * deltaTime;
            profiler.CurrentVelocity += profiler.CurrentAcceleration * deltaTime;
        }
    }
    break;

    case 0:
    {
    }
    break;

    default:
        // Idle
        break;
    }

    // std::cout << "stage: " << profiler.stage << " \n";
    // std::cout << "CurrentVelocity: " << profiler.CurrentVelocity << " \n";
    profiler.CurrentPosition += profiler.CurrentVelocity * deltaTime * profiler.direction;


    prev_cycle_time = discreteTime;
    return nullptr;
}


// Periodic task
void my_task()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_time).count();
    // start_time = now;
    if (profiler.stage == 0)
    {
        profiler.CurrentPosition = profiler.TargetPosition;
        profiler.TargetPosition = posDist(rng);

        profiler.TargetDistance = std::abs(profiler.TargetPosition - profiler.CurrentPosition);
        profiler.direction = (profiler.TargetPosition - profiler.CurrentPosition < 0) ? -1 : 1;

        profiler.MaxVelocity = velDist(rng);
        profiler.MaxAcceleration = accDist(rng);
        profiler.MaxDeceleration = accDist(rng);
        profiler.Jerk = ((profiler.MaxAcceleration + profiler.MaxDeceleration) / 2.0) * 12.5;
        // profiler.CurrentVelocity = 0;
        // profiler.CurrentAcceleration = 0;
        prev_cycle_time = 0.0;
        profiler.CalculatePositionProfile();
        profiler.begin = std::chrono::high_resolution_clock::now();
        profiler.stage = 10;
    }
    positionGenerator();
    // Add a sample to the graph (X=time in ms, Y=example value)
    graph.dataPoints.push_back({static_cast<double>(elapsed) / 1e6, profiler.CurrentPosition});
    // std::cout << "Elapsed time: " << elapsed / 1e6 << " ms\n";
    // Keep buffer size fixed
    if (graph.dataPoints.size() > BufferSize)
        graph.dataPoints.erase(graph.dataPoints.begin());
}

// GLUT callbacks
void displayCallback() { graph.display(); }
void reshapeCallback(int w, int h) { graph.reshape(w, h); }
void mouseCallback(int button, int state, int x, int y) { graph.mouse(button, state, x, y); }
void keyboardCallback(unsigned char key, int x, int y) { graph.keyboard(key, x, y); }
void idleCallback() { glutPostRedisplay(); }

int main(int argc, char **argv)
{
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(graph.winW, graph.winH);
    glutCreateWindow("Live Graph");

    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutMouseFunc(mouseCallback);
    glutKeyboardFunc(keyboardCallback);
    glutIdleFunc(idleCallback);

    glClearColor(1, 1, 1, 1);

    // Start your periodic task in a separate thread
    std::thread([=]()
                {
        update_period(1'000'000);
        run_periodic(my_task); })
        .detach();

    glutMainLoop();
    return 0;
}
