#pragma once

/*
 *	engine.h
 *
 *	Core game engine, containing startup behavior and game loop.
 *
 */

class Engine;

#include"options.h"
#include"window.h"
#include"coreGame.h"


// Fixed FPS update rate
#define NTW_UPDATES_PER_SECOND		60
#define NTW_UPDATE_TIME_MICRO		1000000/NTW_UPDATES_PER_SECOND

// Minimum update rate, game will not update slower than this even if FPS is lower
#define NTW_MIN_UPDATES_PER_SECOND	10
#define NTW_MIN_UPDATE_TIME_MICRO	1000000/NTW_MIN_UPDATES_PER_SECOND


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
};
