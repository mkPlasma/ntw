#pragma once

/*
 *	soundEngine.h
 *
 *	Handles playing SFX and music and audio effects.
 *
 */

class SoundEngine;

#include"core/options.h"
#include"math/vec3.h"
#include"core/world.h"
#include<al.h>
#include<alc.h>

class World;


class SoundEngine{

	SoundOptions& sOptions_;

	ALCdevice* device_;
	ALCcontext* context_;


	World* world_;

	vector<ALuint> worldSourcesPlaying_;


public:
	SoundEngine(SoundOptions& sOptions);

	void init();
	void close();


	void initWorldSound(World* world);
	void playWorldSound();
	void updateWorldSources();

	void addCollisionSounds();


	void setListenerPosition(const Vec3& position);
	void setListenerOrientation(const Vec3& at, const Vec3& up);
};
