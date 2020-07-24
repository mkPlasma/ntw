#pragma once

/*
 *	shaderProgram.h
 *	
 *	Stores an OpenGL shader program.
 *	
 *	Programs include vertex, geometry, and fragment shader.
 *	Shader attributes can also be bound.
 *	
 */

class ShaderProgram;

#include<glad/glad.h>
#include<string>

using std::string;


class ShaderProgram{

	const string name_;

	GLuint program_;
	GLuint vShader_, fShader_;

	GLuint compileShader(const string& path, const GLenum& type);

public:
	ShaderProgram(const string& name);
	ShaderProgram();

	void compile(const string& vShader, const string& fShader);

	void bindAttrib(int index, const string& name);
	void link();

	void use();

	void destroy();

	GLint getUniformLocation(const GLchar* uniform);
	
	const string& getName();
	GLuint getProgram();
};
