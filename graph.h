// graph.h
#pragma once

#include <vector>
#include <string>
#include <utility>
// Include GLFW header instead of GLUT
#include <GLFW/glfw3.h>

class Graph
{
public:
    Graph(int winw = 800, int winh = 600);
    ~Graph();

    // Data remains the same
    int winW = 800, winH = 600;
    std::vector<std::pair<double, double>> dataPoints;

    // The display logic is still ours to control
    void display();

    // --- Member functions to handle events ---
    // These will be called by our static callbacks
    void handleMouseButton(int button, int action, double xpos, double ypos);
    void handleKeyboard(int key);
    void handleTextInput(unsigned int codepoint);
    void setFramebufferSize(int w, int h);
    void drawText(float x, float y, const std::string &text);

    // --- GLFW Callbacks ---
    // These must be static to be used with GLFW's C-style API
    static void framebuffer_size_callback(GLFWwindow *window, int width, int height);
    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
    static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void character_callback(GLFWwindow *window, unsigned int codepoint);

private:
    // We will no longer draw the button or textbox as they required text rendering
    // This simplifies the migration. You can add them back with a real UI
    // or font rendering library.
};