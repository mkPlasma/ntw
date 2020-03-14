#include"soundEngine.h"

#include"core/error.h"
#include<algorithm>

using ntw::fatalError;
using std::min;


SoundEngine::SoundEngine(SoundOptions& sOptions) : sOptions_(sOptions) {

}

void SoundEngine::init(){

	// TEMPORARY
	// Use default sound device
	device_ = alcOpenDevice(NULL);

	if(!device_)
		fatalError("Unable to open default sound device!");


	// Reset error stack
	alGetError();


	// Create audio context
	context_ = alcCreateContext(device_, NULL);

	if(!alcMakeContextCurrent(context_))
		fatalError("Unable to set AL context!");

}

void SoundEngine::close(){
	alcMakeContextCurrent(NULL);
	alcDestroyContext(context_);
	alcCloseDevice(device_);
}


void SoundEngine::initWorldSound(World* world){
	world_ = world;
}

void SoundEngine::playWorldSound(){
	updateWorldSources();
	addCollisionSounds();
}

void SoundEngine::updateWorldSources(){

	// Check if any currently playing sounds have finished
	for(int i = 0; i < worldSourcesPlaying_.size(); i++){

		ALuint source = worldSourcesPlaying_[i];

		ALint state;
		alGetSourcei(source, AL_SOURCE_STATE, &state);

		// Delete source
		if(state != AL_PLAYING && state != AL_PAUSED){
			alDeleteSources(1, &source);
			worldSourcesPlaying_.erase(worldSourcesPlaying_.begin() + i--);
		}
	}
}

void SoundEngine::addCollisionSounds(){

	vector<ContactManifold>& contactManifolds = world_->getPhysicsEngine().getContactManifolds();

	// For each colliding object pair
	for(auto i = contactManifolds.begin(); i != contactManifolds.end(); i++){

		// Get sound IDs
		Material* obj1Material = (*i).objects.object1->getMaterial();
		Material* obj2Material = (*i).objects.object2->getMaterial();
		ALuint obj1Sound = obj1Material == nullptr ? -1 : obj1Material->collisionSound;
		ALuint obj2Sound = obj2Material == nullptr ? -1 : obj2Material->collisionSound;

		// Check that the objects have collision sounds
		if(obj1Sound == -1 && obj2Sound == -1)
			continue;


		// Find new contact with greatest lambda average (greatest impulse applied)
		Contact contact;
		float lambda = 0;

		for(auto j = (*i).contacts.begin(); j != (*i).contacts.end(); j++){
			if((*j).isNew && (*j).lambdaAvg > lambda){
				contact = *j;
				lambda = (*j).lambdaAvg;
			}
		}

		// If a contact was not found, exit
		//if(lambda == 0)
		//	continue;


		// Set volume based on lambda
		float volume = min(lambda / 5, 1.0f);
		volume *= volume;

		volume = 0.2f;

		// Don't play sound if it's too quiet
		if(volume <= 0.01f)
			continue;

		// Add source
		ALuint source;
		alGenSources(1, &source);
		worldSourcesPlaying_.push_back(source);

		// Set position to halfway between contact points
		Vec3 pos = (contact.obj1ContactGlobal + contact.obj2ContactGlobal) / 2;
		alSource3f(source, AL_POSITION, pos[0], pos[2], pos[1]);

		alSourcef(source, AL_GAIN, volume);

		// Play
		// TEMPORARY: using only object 1's sound right now
		alSourcei(source, AL_BUFFER, obj1Sound == -1 ? obj2Sound : obj1Sound);
		alSourcePlay(source);
	}
}


void SoundEngine::setListenerPosition(const Vec3& position){
	alListener3f(AL_POSITION, position[0], position[2], position[1]);
}

void SoundEngine::setListenerOrientation(const Vec3& at, const Vec3& up){
	ALfloat orientation[] = {-at[0], at[2], at[1], -up[0], up[2], up[1]};
	alListenerfv(AL_ORIENTATION, orientation);
}
