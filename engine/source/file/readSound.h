#pragma once

#include"core/paths.h"
#include<vorbis/vorbisfile.h>
#include<vector>

using std::vector;

#define OGG_READ_BUFFER_SIZE 32768


struct Sound{
	vector<char> data;
	ALenum format;
	ALsizei frequency;
};


namespace ntw{

	Sound readSound(const string& path){

		Sound sound;

		// Open ogg file
		FILE* f;
		fopen_s(&f, (PATH_SOUND + path).c_str(), "rb");

		OggVorbis_File file;
		ov_open(f, &file, NULL, 0);


		// Set format
		vorbis_info* info = ov_info(&file, -1);

		if(info->channels == 1)	sound.format = AL_FORMAT_MONO16;
		else					sound.format = AL_FORMAT_STEREO16;
		
		sound.frequency = info->rate;


		// Read data
		char* buffer = new char[OGG_READ_BUFFER_SIZE];
		long bytes;
		int bitstream;

		do{
			bytes = ov_read(&file, buffer, OGG_READ_BUFFER_SIZE, 0, 2, 1, &bitstream);
			sound.data.insert(sound.data.end(), buffer, buffer + bytes);
		} while(bytes > 0);


		delete[] buffer;

		// Close file
		ov_clear(&file);


		return sound;
	}
}
