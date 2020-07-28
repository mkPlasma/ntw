#include"engine.h"

#include"physics/physDefine.h"
#include<iostream>
#include<chrono>
#include<thread>
#include<vector>
#include<algorithm>

#define currentTime std::chrono::high_resolution_clock::now()
#define timeBetween(t1, t2) (int)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()


Engine::Engine() : window_(options_), game_(options_, window_) {

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
	auto lastUpdate = currentTime;
	auto lastPhysicsUpdate = currentTime;

	int lastSecond = 0;
	int frames = 0;

	std::vector<int> waitDifferences;

	while(!glfwWindowShouldClose(winPtr_)){

		// Start time
		auto loopStartTime = currentTime;


		// Get time delta and cap to minimum update rate
		unsigned long long updateTime		= timeBetween(lastUpdate, currentTime);
		unsigned long long physUpdateTime	= timeBetween(lastPhysicsUpdate, currentTime);

		// Cap minimum update rate
		if(updateTime > NTW_MIN_UPDATE_TIME_MICRO)
			updateTime = NTW_MIN_UPDATE_TIME_MICRO;


		float timeDelta		= updateTime		/ 1000000.0f;
		float physTimeDelta = physUpdateTime	/ 1000000.0f;
		
		// Update last update times
		lastUpdate = currentTime;

		// Check whether to update physics
		bool updatePhysics = physUpdateTime >= NTW_PHYS_UPDATE_TIME_MICRO || firstUpdate;

		if(updatePhysics){
			lastPhysicsUpdate = currentTime;
			physTimeDelta = 0;
		}

		// Get game time (ms)
		int timeMillis = timeBetween(startTime, currentTime) / 1000;


		// Update game
		window_.updateKeys();
		game_.update(timeMillis, timeDelta, updatePhysics);

		// Render
		game_.render(timeMillis, physTimeDelta);
		glfwSwapBuffers(winPtr_);

		// Poll window events
		glfwPollEvents();


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

			//std::this_thread::sleep_for(std::chrono::microseconds(NTW_UPDATE_TIME_MICRO - elapsed));

			while(timeBetween(loopStartTime, currentTime) < NTW_UPDATE_TIME_MICRO)
				std::this_thread::sleep_for(std::chrono::microseconds(1));

			int difference = timeBetween(beforeWaitTime, currentTime) - NTW_UPDATE_TIME_MICRO;
			//difference = difference < 0 ? -difference : difference;
			waitDifferences.push_back(difference);
		}

		firstUpdate = false;
	}

	finish();
}

void Engine::finish(){
	game_.finish();
}
