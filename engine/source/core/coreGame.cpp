#include"coreGame.h"

#include"physics/physDefine.h"
#include<math.h>


CoreGame::CoreGame(Options& options, Window& window) :
	options_(options), window_(window), renderer_(options.graphics), soundEngine_(options.sound),
	world_(options_, resCache_, window_, renderer_, soundEngine_) {

	mouseLocked_ = false;
}

void CoreGame::init(){

	renderer_.init();
	soundEngine_.init();
	world_.test();

	renderer_.initWorldRendering(&world_);
	soundEngine_.initWorldSound(&world_);
}

void CoreGame::update(int time, float timeDelta, bool updatePhysics){

	// Lock mouse when window is clicked in
	if(window_.isMouseInWindow() && window_.isMouseButtonDown(GLFW_MOUSE_BUTTON_1)){
		mouseLocked_ = true;
		window_.setMouseLock(true);
		window_.centerMousePosition();
	}
	// Unlock when pause is pressed
	if(window_.isKeyPressed(NTW_KEY_PAUSE)){
		mouseLocked_ = false;
		window_.setMouseLock(false);
	}

	// Refresh shaders
	if(window_.isKeyPressed(NTW_KEY_RELOAD_FILES)){
		renderer_.destroy();
		renderer_.init();
	}

	world_.update(timeDelta, updatePhysics);
}

void CoreGame::render(int time, float physTimeDelta){
	renderer_.renderWorld(time, physTimeDelta);
	renderer_.render(time);
}

void CoreGame::finish(){
	world_.unload();
	renderer_.cleanupWorldRendering();
	renderer_.destroy();
	soundEngine_.close();
}
