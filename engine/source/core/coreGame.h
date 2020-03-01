#pragma once

#include"window.h"
#include"options.h"
#include"graphics/renderer.h"
#include"graphics/textureCache.h"
#include"world.h"


class CoreGame{

	Options& options_;
	Window& window_;

	Renderer renderer_;
	TextureCache texCache_;
	World world_;


	bool mouseLocked_;

	bool noclip_;
	bool noclipHeld_;

	bool rendererRefreshed_;

public:
	CoreGame(Options& options, Window& window);

	void init();

	void update(const int& time, const bool& updatePhysics);
	void render(const int& time, const float& delta);

	void finish();
};
