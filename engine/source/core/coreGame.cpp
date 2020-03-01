#include"coreGame.h"

#include"physics/physDefine.h"
#include<math.h>


CoreGame::CoreGame(Options& options, Window& window) :
	options_(options), window_(window), renderer_(options.graphics), world_(options, texCache_, window) {

	mouseLocked_ = false;

	noclip_ = false;
	noclipHeld_ = false;

	rendererRefreshed_ = false;
}

void CoreGame::init(){

	renderer_.init();
	world_.test();

	renderer_.initWorldRendering(&world_);
}

void CoreGame::update(const int& time, const bool& updatePhysics){

	// Lock mouse when window is clicked in
	if(window_.isMouseInWindow() && window_.isMouseButtonDown(GLFW_MOUSE_BUTTON_1)){
		mouseLocked_ = true;
		window_.setMouseLock(true);
		window_.centerMousePosition();
	}
	// Unlock when pause is pressed
	if(window_.isKeyDown(options_.control.keys[NTW_KEY_PAUSE])){
		mouseLocked_ = false;
		window_.setMouseLock(false);
	}

	// Refresh shaders
	if(window_.isKeyDown(GLFW_KEY_R) && !rendererRefreshed_){
		rendererRefreshed_ = true;
		renderer_.destroy();
		renderer_.init();
	}
	else if(!window_.isKeyDown(GLFW_KEY_R))
		rendererRefreshed_ = false;



	world_.update(updatePhysics);
}

void CoreGame::render(const int& time, const float& delta){
	renderer_.renderWorld(time, delta);
}

void CoreGame::finish(){
	world_.unload();
	renderer_.cleanupWorldRendering();
	renderer_.destroy();
}
