#pragma once

/*
 *	world.h
 *
 *	Game world. Stores map and objects.
 *
 */

class World;

#include"graphics/renderer.h"
#include"physics/physicsEngine.h"
#include"core/resourceCache.h"
#include"graphics/camera.h"
#include"sound/soundEngine.h"
#include"objects/player.h"
#include"objects/portal.h"
#include<vector>
#include<string>

using std::vector;
using std::string;

class Renderer;
class SoundEngine;
class PhysicsEngine;


class World{

	Options& options_;
	ResourceCache& resCache_;
	Window& window_;

	Renderer& renderer_;
	SoundEngine& soundEngine_;

	PhysicsEngine physicsEngine_;

	bool initialized_;

	Camera camera_;
	Player* player_;

	vector<Object*> objects_;
	vector<PhysicsObject*> physicsObjects_;
	vector<Portal*> portals_;

public:
	World(Options& options, ResourceCache& resCache, Window& window, Renderer& renderer, SoundEngine& soundEngine);

	// Temporary test
	void test();

	void update(float timeDelta, bool updatePhysics);

	void unload();


	void addObject(Object* object);
	void removeObject(Object* object);


	PhysicsEngine& getPhysicsEngine();

	Camera& getCamera();

	const vector<Object*>& getObjects();
	const vector<Portal*>& getPortals();
};
