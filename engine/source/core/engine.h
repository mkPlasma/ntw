#pragma once

/*
 *	engine.h
 *
 *	Core game engine, containing startup behavior and game loop.
 *
 */

#include"options.h"
#include"window.h"
#include"coreGame.h"


#define UPDATES_PER_SECOND 60

#define UPDATE_TIME_MICRO       1000000/UPDATES_PER_SECOND


class Engine{

	Options options_;

	Window window_;
	GLFWwindow* winPtr_;

	CoreGame game_;

	void gameLoop();

	void finish();

public:
	Engine();

	void start();

	void render(const int& time, const float& delta);
};
