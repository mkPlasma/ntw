#pragma once

/*
 *	world.h
 *
 *	Game world. Stores map and objects.
 *
 */

#include"physics/physicsEngine.h"
#include"core/resourceCache.h"
#include"graphics/camera.h"
#include"sound/soundEngine.h"
#include"objects/player.h"
#include<vector>
#include<string>

using std::vector;
using std::string;

struct Camera;
class SoundEngine;


class World{

	Options& options_;
	ResourceCache& resCache_;

	SoundEngine& soundEngine_;

	PhysicsEngine physicsEngine_;

	Camera camera_;
	Player* player_;

	vector<Object*> objects_;
	vector<PhysicsObject*> physicsObjects_;

public:
	World(Options& options, ResourceCache& resCache, SoundEngine& soundEngine, Window& window);

	// Temporary test
	void test();

	void update(bool updatePhysics);

	void unload();


	Camera& getCamera();

	vector<Object*>& getObjects();

	PhysicsEngine& getPhysicsEngine();
};
