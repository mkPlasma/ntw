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

#ifdef __NTW_DEBUG__
	bool keys_[NTW_KEYS_DEBUG_SIZE];
	bool keyPress_[NTW_KEYS_DEBUG_SIZE];
#else
	bool keys_[NTW_KEYS_SIZE];
	bool keyPress_[NTW_KEYS_SIZE];
#endif

public:
	Window(Options& options);
	~Window();

	void init();

	void setMouseLock(bool lock);
	void centerMousePosition();

	void updateKeys();
	bool isKeyDown(int key);
	bool isKeyPressed(int key);
	bool isMouseButtonDown(int button);

	int getMouseX();
	int getMouseY();

	bool isMouseInWindow();
	bool isMouseLocked();

	GLFWwindow* getWinPtr();
};
