#pragma once

#include<iostream>
#include<glad/glad.h>
#include<al.h>

using std::string;


struct Material{

	// Graphics
	GLuint texture;
	string shaderProgram;

	// Physics


	// Sound
	ALuint collisionSound;
};
