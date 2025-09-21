#pragma once
#include <vector>
#include <string>
#include <utility>
#include <GL/glut.h>

class Graph
{
public:
    int winW = 800, winH = 600;
    int buttonX = 50, buttonY = 520, buttonW = 120, buttonH = 40;

    // Textbox
    double tx = 200;
    double ty = 520; // align with button
    double tw = 200;
    double th = 40;
    std::string textboxInput = "";
    bool textboxActive = false;

    std::vector<std::pair<double, double>> dataPoints;

    void display();
    void drawButton();
    void drawTextbox();
    void mouse(int button, int state, int x, int y);
    void keyboard(unsigned char key, int, int);
    void reshape(int w, int h);
};
