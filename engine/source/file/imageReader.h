#pragma once

/*
 *	imageReader.h
 *
 *	Reads a PNG file and returns its image data in a Texture object.
 *
 */

#include"binStream.h"
#include"graphics/texture.h"
#include<string>

using std::string;


class ImageReader{

	BinStream file_;

public:

	Texture* read(const string& path);
};
