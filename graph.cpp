// graph.cpp
#include "graph.h"
#include "stb_easy_font.h"
#include <string>
#include <algorithm>
#include <iostream>

// --- Constructor & Destructor ---
Graph::Graph(int winw, int winh) : winW(winw), winH(winh) {}
Graph::~Graph() {}

// --- Event Handlers (Member Functions) ---
void Graph::setFramebufferSize(int w, int h)
{
    winW = w;
    winH = h;
    glViewport(0, 0, w, h);
}

void Graph::handleMouseButton(int button, int action, double xpos, double ypos)
{
    // The button and textbox are removed, so this is now empty.
    // You could add logic for interacting with the graph here.
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        std::cout << "Mouse clicked at: (" << xpos << ", " << winH - ypos << ")" << std::endl;
    }
}

void Graph::handleKeyboard(int key)
{
    // Example: Press escape to close
    if (key == GLFW_KEY_ESCAPE)
    {
        // This is a placeholder; closing is handled in the main loop
        std::cout << "Escape key pressed." << std::endl;
    }
}

void Graph::handleTextInput(unsigned int codepoint)
{
    // Textbox is removed, so this is empty.
    // This is where you would handle text input for a textbox.
}

// --- Static GLFW Callbacks ---
// These are the functions GLFW will call directly.

void Graph::framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    // Retrieve the Graph instance and call its member function
    Graph *graph = static_cast<Graph *>(glfwGetWindowUserPointer(window));
    if (graph)
    {
        graph->setFramebufferSize(width, height);
    }
}

void Graph::mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    Graph *graph = static_cast<Graph *>(glfwGetWindowUserPointer(window));
    if (graph)
    {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        graph->handleMouseButton(button, action, xpos, ypos);
    }
}

void Graph::key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    Graph *graph = static_cast<Graph *>(glfwGetWindowUserPointer(window));
    if (graph && action == GLFW_PRESS)
    {
        graph->handleKeyboard(key);
    }
}

void Graph::character_callback(GLFWwindow *window, unsigned int codepoint)
{
    Graph *graph = static_cast<Graph *>(glfwGetWindowUserPointer(window));
    if (graph)
    {
        graph->handleTextInput(codepoint);
    }
}

// --- Display Function ---
// The core OpenGL drawing logic remains almost identical.
void Graph::display()
{
    // Set up projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, winW, 0, winH, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // X axis values (min and max)
    std::string xminLabel = std::to_string(tmin);
    std::string xmaxLabel = std::to_string(tmax);
    drawText(graphX, graphY - 20, xminLabel.c_str());
    drawText(graphX + graphW - 40, graphY - 20, xmaxLabel.c_str());

    // Y axis values (min and max)
    std::string yminLabel = std::to_string(vmin);
    std::string ymaxLabel = std::to_string(vmax);
    drawText(graphX - 40, graphY, yminLabel.c_str());
    drawText(graphX - 40, graphY + graphH - 10, ymaxLabel.c_str());
    // Clear the screen
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    if (dataPoints.empty())
    {
        return; // Nothing to draw
    }

    // --- Find data min/max for scaling ---
    double tmin = dataPoints.front().first;
    double tmax = dataPoints.back().first;
    if (tmax == tmin)
        tmax += 1.0;

    double vmin = dataPoints.front().second;
    double vmax = vmin;
    for (const auto &p : dataPoints)
    {
        vmin = std::min(vmin, p.second);
        vmax = std::max(vmax, p.second);
    }
    if (vmax == vmin)
        vmax += 1.0;

    // Define graph area (with padding)
    const float padding = 50.0f;
    float graphX = padding;
    float graphY = padding;
    float graphW = winW - 2 * padding;
    float graphH = winH - 2 * padding;

    // --- Draw axes ---
    glColor3f(0, 0, 0);
    glBegin(GL_LINES);
    // Y axis
    glVertex2f(graphX, graphY);
    glVertex2f(graphX, graphY + graphH);
    // X axis
    glVertex2f(graphX, graphY);
    glVertex2f(graphX + graphW, graphY);
    glEnd();

    // --- Draw the graph line ---
    glColor3f(1, 0, 0); // Red line
    glBegin(GL_LINE_STRIP);
    for (const auto &point : dataPoints)
    {
        float x = graphX + (point.first - tmin) / (tmax - tmin) * graphW;
        float y = graphY + (point.second - vmin) / (vmax - vmin) * graphH;
        glVertex2f(x, y);
    }
    glEnd();
}

void drawText(float x, float y, const char *text)
{
    char buffer[99999]; // ~500 chars
    int num_quads;

    num_quads = stb_easy_font_print(x, y, (char *)text, NULL, buffer, sizeof(buffer));
    glColor3f(0, 0, 0); // black text
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, buffer);
    glDrawArrays(GL_QUADS, 0, num_quads * 4);
    glDisableClientState(GL_VERTEX_ARRAY);
}
