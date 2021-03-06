#pragma once

#include"window.h"
#include"keys.h"
#include<vector>

using std::vector;


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
		keys[NTW_KEY_FORWARDS]		= GLFW_KEY_W;
		keys[NTW_KEY_BACKWARDS]		= GLFW_KEY_S;
		keys[NTW_KEY_RIGHT]			= GLFW_KEY_D;
		keys[NTW_KEY_LEFT]			= GLFW_KEY_A;

		keys[NTW_KEY_JUMP]			= GLFW_KEY_SPACE;

		keys[NTW_KEY_GRAB]			= GLFW_KEY_E;

		keys[NTW_KEY_PAUSE]			= GLFW_KEY_ESCAPE;


		keys[NTW_KEY_NOCLIP] = GLFW_KEY_V;

		keys[NTW_KEY_RELOAD_FILES]	= GLFW_KEY_R;
	}
};


struct GraphicsOptions{
	int resolutionX;
	int resolutionY;

	bool fullscreen;
	bool useVSync;

	int fov;

	int msaaSamples;


	// Defaults
	GraphicsOptions() :
		resolutionX	(640),
		resolutionY	(480),
		fullscreen	(true),
		useVSync	(true),
		fov			(90),
		msaaSamples	(4)
	{}
};


struct SoundOptions{
	float sfxVolume;
	float musicVolume;


	// Defaults
	SoundOptions() :
		sfxVolume	(1.0f),
		musicVolume	(1.0f)
	{}
};


struct Options{
	ControlOptions control;
	GraphicsOptions graphics;
	SoundOptions sound;
};
