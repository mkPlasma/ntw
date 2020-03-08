#pragma once

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

	bool noclip_;
	bool noclipHeld_;

	bool rendererRefreshed_;

public:
	CoreGame(Options& options, Window& window);

	void init();

	void update(int time, bool updatePhysics);
	void render(int time, float delta);

	void finish();
};
