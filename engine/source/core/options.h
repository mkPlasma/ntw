#pragma once

#include"window.h"
#include<vector>

using std::vector;


enum NTWKeys{
	NTW_KEY_FORWARDS,
	NTW_KEY_BACKWARDS,
	NTW_KEY_RIGHT,
	NTW_KEY_LEFT,
	
	NTW_KEY_JUMP,
	NTW_KEY_NOCLIP,

	NTW_KEY_PAUSE,

	NTW_KEYS_SIZE
};


struct ControlOptions{

	vector<int> keys;
	vector<bool> keyBound;

	float mouseSensitivity;
	

	// Defaults
	ControlOptions() :
		keys(NTW_KEYS_SIZE, 0),
		keyBound(NTW_KEYS_SIZE, true),

		mouseSensitivity(0.06f)
	{
		keys[NTW_KEY_FORWARDS]	= GLFW_KEY_W;
		keys[NTW_KEY_BACKWARDS]	= GLFW_KEY_S;
		keys[NTW_KEY_RIGHT]		= GLFW_KEY_D;
		keys[NTW_KEY_LEFT]		= GLFW_KEY_A;

		keys[NTW_KEY_JUMP]		= GLFW_KEY_SPACE;
		keys[NTW_KEY_NOCLIP]	= GLFW_KEY_V;

		keys[NTW_KEY_PAUSE]		= GLFW_KEY_ESCAPE;
	}
};


struct GraphicsOptions{
	int resolutionX;
	int resolutionY;

	bool fullscreen;
	bool useVSync;

	int fov;


	// Defaults
	GraphicsOptions() :
		fullscreen	(true),
		useVSync	(true),
		fov			(60)
	{}
};


struct Options{
	ControlOptions control;
	GraphicsOptions graphics;
};
