#include"world.h"

#include"objects/modelFunc.h"

using ntw::setModelProperties;


World::World(Options& options, ResourceCache& resCache, SoundEngine& soundEngine, Window& window) : options_(options), resCache_(resCache),
	soundEngine_(soundEngine), physicsEngine_(objects_, physicsObjects_), player_(new Player(options_.control, window)) {

}

void World::test(){

	// Texture
	Model* testModel = new Model(ntw::getCube(false));
	setModelProperties(testModel);
	Material* testMaterial = new Material();
	testMaterial->texture = resCache_.loadTexture("tile.bmp");
	testMaterial->collisionSound = resCache_.loadSound("test2.ogg");

	/*
	const int s = 32;

	for(int x = -s / 2; x < s / 2; x++){
		for(int y = -s / 2; y < s / 2; y++){
			for(int z = -s / 2; z < s / 2; z++){
				Object* object = new Object(testModel, RenderType::STATIC, HitboxType::MESH);
				object->setPosition(x * 2, y * 2, z * 2);
				object->setScale(0.2);
				objects_.push_back(object);
			}
		}
	}
	*/

	Object* obj1 = new Object(testModel, testMaterial, RenderType::STATIC, HitboxType::MESH);
	obj1->setPosition(-3, 0, 0);
	obj1->setScale(0.5);
	obj1->setRotation(15, 15, 15);
	objects_.push_back(obj1);

	Object* obj2 = new Object(testModel, testMaterial, RenderType::STATIC, HitboxType::MESH);
	obj2->setPosition(3, 0, 0);
	obj2->setScale(1);
	//obj2->setRotation(0, 0, 45);
	objects_.push_back(obj2);


	Model* sphereModel = new Model(ntw::getSphere(16, 16, true));
	setModelProperties(sphereModel);
	Material* sphereMaterial = new Material();
	sphereMaterial->texture = resCache_.loadTexture("checker.bmp");

	Object* obj3 = new Object(sphereModel, sphereMaterial, RenderType::STATIC, HitboxType::PREDEFINED);
	obj3->setPosition(0, -2, 0.2f);
	objects_.push_back(obj3);


	Model* floorModel = new Model(ntw::getCube());
	setModelProperties(floorModel);
	Material* floorMaterial = new Material();
	floorMaterial->texture = resCache_.loadTexture("tile.bmp");

	for(auto i = floorModel->texCoords.begin(); i != floorModel->texCoords.end(); i++)
		*i *= 50;

	Object* floor = new Object(floorModel, floorMaterial, RenderType::STATIC, HitboxType::MESH);
	floor->setPosition(0, 0, -2);
	floor->setScale(25, 25, 1);
	objects_.push_back(floor);


	const int c1 = 2;
	const int c2 = 2;

	for(float x = 0; x < c1; x++){
		for(float y = 0; y < c1; y++){
			for(float z = 0; z < c2; z++){
				PhysicsObject* physObj1 = new PhysicsObject(testModel, testMaterial, PhysicsType::DYNAMIC_SIMPLE, HitboxType::MESH, 1);
				physObj1->setPosition(0 + x - c1 / 2, 4 + y - c1 / 2, z  * 0.75f);
				//physObj1->rotate(0, 30, 0);
				//physObj1->rotate(0, 30, 0);
				//physObj1->setScale(0.5f, 0.5f, 0.1f);
				physObj1->setScale(0.25f);
				objects_.push_back(physObj1);
				physicsObjects_.push_back(physObj1);
			}
		}
	}

	Object* obj4 = new Object(testModel, testMaterial, RenderType::STATIC, HitboxType::MESH);
	obj4->setPosition(0, 4, -1);
	obj4->setScale(4, 0.1f, 4);
	obj4->setRotation(-45, 0, 0);
	//objects_.push_back(obj4);

	PhysicsObject* physObj2 = new PhysicsObject(testModel, testMaterial, PhysicsType::DYNAMIC, HitboxType::MESH, 1);
	physObj2->setPosition(0, 3, 0);
	physObj2->setRotation(90, 30, 0);
	physObj2->setScale(0.3f);
	//objects_.push_back(physObj2);
	//physicsObjects_.push_back(physObj2);

	//((PhysicsObject*)physObj2)->setVelocity(Vec3(0, -3, 0));


	//Object* physObj2 = new Object(testModel, testMaterial, RenderType::DYNAMIC, PhysicsType::SEMI_DYNAMIC, HitboxType::MESH);
	//physObj2->setPosition(0, 3, -0.25f);
	//physObj2->setScale(0.5f);
	//objects_.push_back(physObj2);

	// Initialize player and add to object lists
	player_->initPlayer();

	objects_.push_back(player_);
	physicsObjects_.push_back(player_);

	// Initialize physics engine
	physicsEngine_.init();
}

void World::update(float timeDelta, bool fullPhysicsUpdate){
	
	// Update player movement
	player_->updatePlayer(timeDelta);

	// Update all objects
	for(Object* obj : objects_)
		obj->update(timeDelta);

	// Update physics
	physicsEngine_.update(timeDelta, fullPhysicsUpdate);

	// Set sound listener and orientation to player's
	soundEngine_.setListenerPosition(player_->getPosition());
	soundEngine_.setListenerOrientation(player_->getLookVector(), player_->getLookUpVector());

	// Play world sounds, including physics sounds
	//soundEngine_.playWorldSound();

	// Set camera to player orientation after player physics update
	camera_.position	= player_->getPosition();

	// Eye level
	camera_.position[2] = camera_.position[2] - player_->getScale()[2] + 0.7f;

	camera_.velocity	= player_->getVelocity();
	camera_.yaw			= player_->getYaw();
	camera_.pitch		= player_->getPitch();

}

void World::unload(){

	// Delete models and materials
	// TEMPORARY: need to cache these when map saving is added
	// for now memory is not freed since the world is always destroyed on program close
	/*
	for(auto i = objects_.begin(); i != objects_.end(); i++){
		Model* model		= (*i)->getModel();
		Material* material	= (*i)->getMaterial();

		if(model != nullptr)
			delete model;

		if(material != nullptr){
			resCache_.unloadTexture(material->texture);
			delete material;
		}

		(*i)->setModel(nullptr);
		(*i)->setMaterial(nullptr);
	}
	*/

	objects_.clear();
}


Camera& World::getCamera(){
	return camera_;
}

vector<Object*>& World::getObjects(){
	return objects_;
}

PhysicsEngine& World::getPhysicsEngine(){
	return physicsEngine_;
}
