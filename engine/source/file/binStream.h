#pragma once

/*
 *	binStream.h
 *
 *	Binary file reader and writer.
 *
 */

#include<fstream>
#include<string>

using std::fstream;
using std::ios;
using std::string;


class BinStream{

	fstream file_;
	char* buffer_;

public:

	BinStream(int bufferSize = 8);

	void open(const string& path);
	void close();

	bool isOpen();


	// Seek forward based on current position
	void seek(int numBytes);

	// Seek from start of file
	void seekTo(int numBytes);


	char* readBytes(int numBytes);
	int readInt();
};
