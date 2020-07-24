#pragma once

/*
 *	window.h
 *
 *	Game window.
 *
 */

class Window;

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include"core/error.h"
#include"options.h"
#include"keys.h"

struct Options;


class Window{

	GLFWwindow* window_;
	Options& options_;

	int centerX_;
	int centerY_;
	bool mouseLocked_;

	bool keys_[NTW_KEYS_SIZE];
	bool keyPress_[NTW_KEYS_SIZE];

public:
	Window(Options& options);
	~Window();

	void init();

	void updateKeys();

	void setMouseLock(bool lock);
	void centerMousePosition();


	bool isKeyDown(int key);
	bool isKeyPressed(int key);
	bool isMouseButtonDown(int button);

	int getMouseX();
	int getMouseY();

	bool isMouseInWindow();
	bool isMouseLocked();

	GLFWwindow* getWinPtr();
};
