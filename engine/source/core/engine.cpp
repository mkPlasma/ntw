#include"engine.h"

#include"physics/physDefine.h"
#include<iostream>
#include<chrono>
#include<thread>
#include<vector>

#define currentTime std::chrono::high_resolution_clock::now()
#define timeBetween(t1, t2) (int)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()


Engine::Engine() : window_(options_.graphics), game_(options_, window_) {

}

void Engine::start(){

	// TEMPORARY
	// Default resolution
	options_.graphics.resolutionX = 1280;
	options_.graphics.resolutionY = 960;
	options_.graphics.fullscreen = false;
	options_.graphics.useVSync = true;

	window_.init();
	winPtr_ = window_.getWinPtr();

	game_.init();

	gameLoop();
}

void Engine::gameLoop(){

	bool firstUpdate = true;

	auto startTime = currentTime;
	auto lastPhysicsUpdate = currentTime;

	int lastSecond = 0;
	int frames = 0;

	std::vector<int> waitDifferences;

	while(!glfwWindowShouldClose(winPtr_)){

		// Start time
		auto loopStartTime = currentTime;


		// Check whether to update physics
		bool updatePhysics = timeBetween(lastPhysicsUpdate, currentTime) >= PHYS_UPDATE_TIME_MICRO || firstUpdate;

		if(updatePhysics)
			lastPhysicsUpdate = currentTime;

		// Update game
		game_.update(timeBetween(startTime, currentTime) / 1000, updatePhysics);
		
		// Render, giving time in milliseconds
		render(timeBetween(startTime, currentTime) / 1000, timeBetween(lastPhysicsUpdate, currentTime) / (float)PHYS_UPDATE_TIME_MICRO);


		// Print FPS
		int currentSecond = timeBetween(startTime, loopStartTime) / 1000000;
		frames++;

		if(currentSecond > lastSecond){

			//system("cls");

			std::cout << "FPS: " << frames << std::endl;
			frames = 0;
			lastSecond = currentSecond;

			if(!options_.graphics.useVSync){
				int avg = 0;

				for(auto i = waitDifferences.begin(); i != waitDifferences.end(); i++)
					avg += *i;

				avg = (int)((float)avg / waitDifferences.size());
				waitDifferences.clear();

				std::cout << "Avg. sleep time discrepancy (us): " << avg << std::endl;
			}
		}

		// Wait for next frame (disabled if vsync is on)
		if(!options_.graphics.useVSync){
			// Get time elapsed during loop
			auto elapsed = timeBetween(loopStartTime, currentTime);

			auto beforeWaitTime = currentTime;

			//std::this_thread::sleep_for(std::chrono::microseconds(UPDATE_TIME_MICRO - elapsed));

			while(timeBetween(loopStartTime, currentTime) < UPDATE_TIME_MICRO)
				std::this_thread::sleep_for(std::chrono::microseconds(1));

			int difference = timeBetween(beforeWaitTime, currentTime) - UPDATE_TIME_MICRO;
			//difference = difference < 0 ? -difference : difference;
			waitDifferences.push_back(difference);
		}

		firstUpdate = false;
	}

	finish();
}

void Engine::render(const int& time, const float& delta){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	game_.render(time, delta);

	glfwSwapBuffers(winPtr_);
	glfwPollEvents();
}

void Engine::finish(){
	game_.finish();
}
