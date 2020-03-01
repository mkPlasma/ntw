#pragma once

/*
 *	textureCache.h
 *
 *	Stores textures that are currently in use.
 *
 */

#include"file/imageReader.h"
#include<vector>

using std::vector;


class TextureCache{

    vector<Texture*> cache_;

    ImageReader imgReader_;

public:

    // Load a texture if not cached, or return cached texture
    Texture* loadTexture(const string& path);

    // Delete texture and remove from cache
    void unloadTexture(Texture* texture);
};
