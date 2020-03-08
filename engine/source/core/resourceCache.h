#pragma once

/*
 *	resourceCache.h
 *
 *	Stores textures that are currently in use.
 *
 */

#include<glad/glad.h>
#include<al.h>
#include<unordered_map>
#include<string>

using std::unordered_map;
using std::string;


class ResourceCache{

	unordered_map<string, GLuint> textures_;
	unordered_map<string, ALuint> sounds_;

public:

	// Load a texture if not cached, or return cached texture
	GLuint loadTexture(const string& path);

	// Delete texture and remove from cache
	void unloadTexture(const GLuint& textureID);

	ALuint loadSound(const string& path);
	void unloadSound(const ALuint& soundID);
};
