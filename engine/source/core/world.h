#pragma once

/*
 *	world.h
 *
 *	Game world. Stores map and objects.
 *
 */

#include"physics/physicsEngine.h"
#include"graphics/textureCache.h"
#include"graphics/camera.h"
#include"objects/player.h"
#include<vector>
#include<string>

using std::vector;
using std::string;

struct Camera;


class World{

	Options& options_;
	TextureCache& texCache_;

	PhysicsEngine physicsEngine_;

	Camera camera_;
	Player* player_;

	vector<Object*> objects_;
	vector<PhysicsObject*> physicsObjects_;

	// Textures that are in use
	vector<Texture*> textures_;

public:
	World(Options& options, TextureCache& texCache, Window& window);

	// Temporary test
	void test();

	void update(const bool& updatePhysics);

	void unload();


	Camera& getCamera();

	vector<Object*>& getObjects();
	vector<Texture*>& getTextures();

	PhysicsEngine& getPhysicsEngine();
};
