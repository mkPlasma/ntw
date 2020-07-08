#pragma once

#include"core/paths.h"
#include"file/binStream.h"


struct Texture{
	char* data;
	int width;
	int height;
};


namespace ntw{
	Texture readImage(const string& path){

		BinStream file;

		file.open(PATH_TEXTURE + path);

		Texture tex;

		// Get dimensions
		file.seekTo(18);
		tex.width = file.readInt();
		tex.height = file.readInt();

		// Read image data
		char* data = new char[(size_t)tex.width * (size_t)tex.height * 4];
		file.seekTo(54);

		for(int y = 0; y < tex.height; y++){
			for(int x = 0; x < tex.width; x++){

				// Read pixel
				char* buffer = file.readBytes(3);

				// Write pixel to texture (including alpha byte)
				for(int i = 0; i < 4; i++)
					data[(tex.width * ((tex.height - 1) - y) + x) * 4 + i] = i < 3 ? buffer[2 - i] : 255;
			}

			// Skip padding
			int p = 4 - ((tex.width * 3) % 4);
			p = p == 4 ? 0 : p;

			file.seek(p);
		}

		file.close();

		tex.data = data;

		return tex;
	}
}