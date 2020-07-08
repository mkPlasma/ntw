#pragma once

/*
 *	coreGame.h
 *
 *	Core game, handles world, rendering, and sound.
 *
 */

class CoreGame;

#include"window.h"
#include"options.h"
#include"resourceCache.h"
#include"graphics/renderer.h"
#include"sound/soundEngine.h"
#include"world.h"


class CoreGame{

	Options& options_;
	Window& window_;

	ResourceCache resCache_;

	Renderer renderer_;
	SoundEngine soundEngine_;

	World world_;


	bool mouseLocked_;

public:
	CoreGame(Options& options, Window& window);

	void init();

	void update(int time, float timeDelta, bool updatePhysics);
	void render(int time, float delta);

	void finish();
};
