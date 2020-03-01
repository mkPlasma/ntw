#include"textureCache.h"


Texture* TextureCache::loadTexture(const string& path){

	// If texture is already cached, return it
	for(auto i = cache_.begin(); i != cache_.end(); i++)
		if((*i)->path == path)
			return *i;


	// Load texture and cache it
	Texture* tex = imgReader_.read(path);
	cache_.push_back(tex);

	return tex;
}

void TextureCache::unloadTexture(Texture* texture){

	// Delete and remove from cache
	for(int i = 0; i < cache_.size(); i++){
		if(cache_[i] == texture){
			delete cache_[i];
			cache_.erase(cache_.begin() + i);
			i--;
		}
	}
}
