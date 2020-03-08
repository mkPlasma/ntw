#pragma once

/*
 *	window.h
 *
 *	Game window.
 *
 */

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include"core/error.h"
#include"options.h"

struct GraphicsOptions;


class Window{

    GLFWwindow* window_;
    GraphicsOptions& gOptions_;

    int centerX_;
    int centerY_;
    bool mouseLocked_;

public:
    Window(GraphicsOptions& gOptions);
    ~Window();

    void init();

    void setMouseLock(bool lock);
    void centerMousePosition();

    bool isKeyDown(int key);
    bool isMouseButtonDown(int button);

    int getMouseX();
    int getMouseY();

    bool isMouseInWindow();
    bool isMouseLocked();

    GLFWwindow* getWinPtr();
};
