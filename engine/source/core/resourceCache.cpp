#include"resourceCache.h"

#include"file/readImage.h"
#include"file/readSound.h"
#include"core/error.h"


GLuint ResourceCache::loadTexture(const string& path){

	// If texture is already cached, return it
	auto i = textures_.find(path);

	if(i != textures_.end())
		return i->second;


	// Load texture
	Texture tex = ntw::readImage(path);

	// Create texture
	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);

	// Texture interpolation
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	// Write data
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.width, tex.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, 0);


	// Texture data is written, so it is no longer necessary
	delete[] tex.data;

	// Cache
	textures_.emplace(path, textureID);

	return textureID;
}

void ResourceCache::unloadTexture(const GLuint& textureID){

	// Check if texture is cached
	for(auto i = textures_.begin(); i != textures_.end(); i++){
		if(i->second == textureID){
			// Delete
			glDeleteTextures(1, &textureID);

			// Remove from cache
			textures_.erase(i);
			return;
		}
	}
}

ALuint ResourceCache::loadSound(const string& path){

	// If sound is already cached, return it
	auto i = sounds_.find(path);

	if(i != sounds_.end())
		return i->second;


	// Load sound
	Sound sound = ntw::readSound(path);

	// Create sound
	ALuint soundID;
	alGenBuffers(1, &soundID);

	// Write data
	alBufferData(soundID, sound.format, sound.data.data(), static_cast<ALsizei>(sound.data.size()), sound.frequency);


	// Cache
	sounds_.emplace(path, soundID);

	return soundID;
}

void ResourceCache::unloadSound(const ALuint& soundID){

	// Check if sound is cached
	for(auto i = sounds_.begin(); i != sounds_.end(); i++){
		if(i->second == soundID){
			// Delete
			alDeleteBuffers(1, &soundID);

			// Remove from cache
			sounds_.erase(i);
			return;
		}
	}
}
