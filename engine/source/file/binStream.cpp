#include"binStream.h"


BinStream::BinStream(int bufferSize) : buffer_(new char[bufferSize]) {

}


void BinStream::open(const string& path){
	file_.open(path, ios::binary | ios::in | ios::out);
}

void BinStream::close(){
	file_.close();
}

bool BinStream::isOpen(){
	return file_.is_open();
}


void BinStream::seek(int numBytes){
	file_.seekg(numBytes, ios::cur);
}

void BinStream::seekTo(int numBytes){
	file_.seekg(numBytes, ios::beg);
}


char* BinStream::readBytes(int numBytes){
	file_.read(buffer_, numBytes);
	return buffer_;
}

int BinStream::readInt(){
	file_.read(buffer_, 4);
	return *reinterpret_cast<int*>(buffer_);
}
