#include"imageReader.h"

#include"paths.h"
#include<iostream>

Texture* ImageReader::read(const string& path){

	file_.open(TEXTURE_PATH + path);

	Texture* tex = new Texture();
	tex->path = path;

	// Get dimensions
	file_.seekTo(18);
	tex->width	= file_.readInt();
	tex->height	= file_.readInt();

	// Read image data
	char* data = new char[(long long)tex->width * (long long)tex->height * 4];
	file_.seekTo(54);

	for(int y = 0; y < tex->height; y++){
		for(int x = 0; x < tex->width; x++){

			// Read pixel
			char* buffer = file_.readBytes(3);

			// Write pixel to texture (including alpha byte)
			for(int i = 0; i < 4; i++){
				data[(tex->width * ((tex->height - 1) - y) + x) * 4 + i] = i < 3 ? buffer[2 - i] : 255;
			}
		}

		// Skip padding
		int p = 4 - ((tex->width * 3) % 4);
		p = p == 4 ? 0 : p;

		file_.seek(p);
	}

	file_.close();

	tex->data = data;

	return tex;
}
