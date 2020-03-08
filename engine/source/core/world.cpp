#include"world.h"

#include"graphics/modelFunc.h"


World::World(Options& options, ResourceCache& resCache, SoundEngine& soundEngine, Window& window) : options_(options), resCache_(resCache),
	soundEngine_(soundEngine), physicsEngine_(objects_, physicsObjects_), player_(new Player(options_.control, window)) {

}

void World::test(){

	// Texture
	Model* testModel = new Model(ntw::getCube(false));
	Material* testMaterial = new Material();
	testMaterial->texture = resCache_.loadTexture("star3.bmp");
	testMaterial->collisionSound = resCache_.loadSound("test2.ogg");

	/*
	const int s = 32;

	for(int x = -s / 2; x < s / 2; x++){
		for(int y = -s / 2; y < s / 2; y++){
			for(int z = -s / 2; z < s / 2; z++){
				Object* object = new Object(testModel, RenderType::STATIC, HitboxType::CUBE);
				object->setPosition(x * 2, y * 2, z * 2);
				object->setScale(0.2);
				objects_.push_back(object);
			}
		}
	}
	*/

	Object* obj1 = new Object(testModel, testMaterial, RenderType::STATIC, HitboxType::CUBE);
	obj1->setPosition(-3, 0, 0);
	obj1->setScale(0.5);
	obj1->setRotation(15, 15, 15);
	objects_.push_back(obj1);

	Object* obj2 = new Object(testModel, testMaterial, RenderType::STATIC, HitboxType::CUBE);
	obj2->setPosition(Vec3(3, 0, 0));
	objects_.push_back(obj2);


	Model* sphereModel = new Model(ntw::getSphere(16, 16, true));
	Material* sphereMaterial = new Material();
	sphereMaterial->texture = resCache_.loadTexture("earth.bmp");

	Object* obj3 = new Object(sphereModel, sphereMaterial, RenderType::STATIC, HitboxType::MESH);
	obj3->setPosition(0, -2, 0.2f);
	objects_.push_back(obj3);


	Model* floorModel = new Model(ntw::getCube());
	Material* floorMaterial = new Material();
	floorMaterial->texture = resCache_.loadTexture("tile.bmp");

	for(auto i = floorModel->texCoords.begin(); i != floorModel->texCoords.end(); i++)
		*i *= 50;

	Object* floor = new Object(floorModel, floorMaterial, RenderType::STATIC, HitboxType::CUBE);
	floor->setPosition(0, 0, -2);
	floor->setScale(25, 25, 1);
	objects_.push_back(floor);


	const int c1 = 1;
	const int c2 = 3;

	for(float x = 0; x < c1; x++){
		for(float y = 0; y < c1; y++){
			for(float z = 0; z < c2; z++){
				PhysicsObject* physObj1 = new PhysicsObject(testModel, testMaterial, HitboxType::CUBE, 1);
				physObj1->setPosition(0 + x - c1 / 2, 4 + y - c1 / 2, -0.5f + z  * 0.51f);
				//physObj1->rotate(45, 0, 0);
				//physObj1->rotate(0, 30, 0);
				//physObj1->setScale(0.5f, 0.5f, 0.1f);
				physObj1->setScale(0.25f);
				physObj1->initPhysics();
				objects_.push_back(physObj1);
				physicsObjects_.push_back(physObj1);
			}
		}
	}

	//Object* physObj2 = new Object(testModel, testMaterial, RenderType::DYNAMIC, PhysicsType::SEMI_DYNAMIC, HitboxType::CUBE);
	//physObj2->setPosition(0, 3, -0.25f);
	//physObj2->setScale(0.5f);
	//objects_.push_back(physObj2);

	// Initialize player and add to object lists
	player_->initPlayer();
	player_->initPhysics();

	objects_.push_back(player_);
	physicsObjects_.push_back(player_);

	// Initialize physics engine
	physicsEngine_.init();
}

void World::update(bool updatePhysics){
	
	// Update player movement
	player_->updatePlayer(updatePhysics);

	// Update physics on required frames
	if(updatePhysics)
		physicsEngine_.update();


	// Set sound listener and orientation to player's
	soundEngine_.setListenerPosition(player_->getPosition());
	soundEngine_.setListenerOrientation(player_->getLookVector(), player_->getLookUpVector());

	soundEngine_.playWorldSound();


	// Set camera to player orientation after player physics update
	camera_.position	= player_->getPosition();
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
