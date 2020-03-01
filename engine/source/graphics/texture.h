#pragma once

/*
 *	texture.h
 *
 *	Stores image data and its dimensions.
 *
 */

#include<string>

using std::string;


struct Texture{
    string path;
	char* data;
	int width;
	int height;

    ~Texture(){
        delete data;
    }
};
