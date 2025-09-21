#include "graph.h"
#include <GL/glut.h>
#include <string>
#include <algorithm>

// Helper to draw text
void drawString(double x, double y, const std::string &str)
{
    glRasterPos2f(x, y);
    for (char c : str)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
}

// --- Draw the Add Value button ---
void Graph::drawButton()
{
    glColor3f(0.2f, 0.6f, 0.8f);
    glBegin(GL_QUADS);
    glVertex2f(buttonX, buttonY);
    glVertex2f(buttonX + buttonW, buttonY);
    glVertex2f(buttonX + buttonW, buttonY + buttonH);
    glVertex2f(buttonX, buttonY + buttonH);
    glEnd();

    glColor3f(1, 1, 1);
    drawString(buttonX + 20, buttonY + 25, "Add Value");
}

// --- Draw the textbox ---
void Graph::drawTextbox()
{
    double tx = 200, ty = 520, tw = 200, th = 40;
    glColor3f(0.9f, 0.9f, 0.9f);
    glBegin(GL_QUADS);
    glVertex2f(tx, ty);
    glVertex2f(tx + tw, ty);
    glVertex2f(tx + tw, ty + th);
    glVertex2f(tx, ty + th);
    glEnd();

    glColor3f(0, 0, 0);
    drawString(tx + 10, ty + 25, textboxInput + (textboxActive ? "_" : ""));
}

// --- Display callback ---
void Graph::display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    drawButton();
    drawTextbox();

    if (!dataPoints.empty())
    {
        // --- Find data min/max for scaling ---
        double tmin = dataPoints.front().first;
        double tmax = dataPoints.back().first;
        if (tmax == tmin)
            tmax += 1.0;

        double vmin = dataPoints.front().second;
        double vmax = vmin;
        for (auto &p : dataPoints)
        {
            vmin = std::min(vmin, p.second);
            vmax = std::max(vmax, p.second);
        }
        if (vmax == vmin)
            vmax += 1.0;

        // --- Draw axes ---
        glColor3f(0, 0, 0);
        glBegin(GL_LINES);
        // Y axis
        glVertex2f(50, 50);
        glVertex2f(50, 500);
        glEnd();

        // X axis (dynamic: at y=0 if in range, otherwise bottom)
        float y0;
        if (vmin <= 0 && vmax >= 0)
            y0 = 50 + ((0 - vmin) / (vmax - vmin)) * 450.0f;
        else
            y0 = 50; // bottom edge if 0 is not visible
        glBegin(GL_LINES);
        glVertex2f(50, y0);
        glVertex2f(750, y0);
        glEnd();

        // --- X axis ticks ---
        for (int i = 0; i <= 10; i++)
        {
            float x = 50 + i * 70.0f;
            glBegin(GL_LINES);
            glVertex2f(x, y0 - 5);
            glVertex2f(x, y0 + 5);
            glEnd();
            double tval = tmin + (tmax - tmin) * i / 10.0;
            drawString(x - 10, y0 - 20, std::to_string((int)tval));
        }

        // --- Y axis ticks ---
        for (int i = 0; i <= 5; i++)
        {
            float y = 50 + i * 90.0f;
            glBegin(GL_LINES);
            glVertex2f(45, y);
            glVertex2f(55, y);
            glEnd();
            double yval = vmin + (vmax - vmin) * i / 5.0;
            drawString(10, y - 5, std::to_string((int)yval));
        }

        // --- Draw the graph line ---
        glColor3f(1, 0, 0);
        glBegin(GL_LINE_STRIP);
        for (auto &point : dataPoints)
        {
            float x = 50 + (point.first - tmin) / (tmax - tmin) * 700.0f;
            float y = 50 + ((point.second - vmin) / (vmax - vmin)) * 450.0f;
            glVertex2f(x, y);
        }
        glEnd();
    }

    glutSwapBuffers();
}

// --- Mouse handler ---
void Graph::mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        int glY = winH - y;

        // Button click
        if (x >= buttonX && x <= buttonX + buttonW &&
            glY >= buttonY && glY <= buttonY + buttonH)
        {
            try
            {
                double val = std::stod(textboxInput);
                double t = dataPoints.empty() ? 0.0 : dataPoints.back().first + 0.05;
                dataPoints.push_back({t, val});
                if (dataPoints.size() > 200)
                    dataPoints.erase(dataPoints.begin());
                textboxInput.clear();
            }
            catch (...)
            {
            }
        }

        // Textbox click
        if (x >= 200 && x <= 400 && glY >= 520 && glY <= 560)
            textboxActive = true;
        else
            textboxActive = false;
    }
    glutPostRedisplay();
}

// --- Keyboard handler ---
void Graph::keyboard(unsigned char key, int, int)
{
    if (textboxActive)
    {
        if (key == 13) // Enter
        {
            try
            {
                double val = std::stod(textboxInput);
                double t = dataPoints.empty() ? 0.0 : dataPoints.back().first + 0.05;
                dataPoints.push_back({t, val});
                if (dataPoints.size() > 200)
                    dataPoints.erase(dataPoints.begin());
                textboxInput.clear();
            }
            catch (...)
            {
            }
        }
        else if (key == 8 || key == 127) // Backspace
        {
            if (!textboxInput.empty())
                textboxInput.pop_back();
        }
        else if (key >= 32 && key <= 126)
        {
            textboxInput.push_back(key);
        }
    }
    glutPostRedisplay();
}

// --- Reshape callback ---
void Graph::reshape(int w, int h)
{
    winW = w;
    winH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}
