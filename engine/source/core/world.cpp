#include"world.h"

#include"objects/modelFunc.h"

using ntw::setModelProperties;


World::World(Options& options, ResourceCache& resCache, Window& window, Renderer& renderer, SoundEngine& soundEngine) :
	options_(options), resCache_(resCache), window_(window), renderer_(renderer), soundEngine_(soundEngine),
	physicsEngine_(*this, objects_, physicsObjects_), initialized_(false) {

}

void World::test(){

	// Texture
	Model* testModel = new Model(ntw::getCube(false));
	setModelProperties(testModel);
	Material* testMaterial = new Material();
	testMaterial->texture = resCache_.loadTexture("tile.bmp");
	testMaterial->shaderProgram = "standard";
	testMaterial->collisionSound = resCache_.loadSound("test2.ogg");

	Material* boxMaterial = new Material();
	boxMaterial->texture = resCache_.loadTexture("box.bmp");
	boxMaterial->shaderProgram = "standard";

	Model* sphereModel = new Model(ntw::getSphere(16, 16, true));
	setModelProperties(sphereModel);
	Material* sphereMaterial = new Material();
	sphereMaterial->texture = resCache_.loadTexture("checker.bmp");
	sphereMaterial->shaderProgram = "standard";

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

	Object* obj1 = new Object(*this, sphereModel, sphereMaterial);
	obj1->setPosition(-3, 0, 0);
	obj1->setScale(0.5);
	obj1->setRotation(15, 15, 15);
	//addObject(obj1);

	Object* obj2 = new Object(*this, testModel, testMaterial);
	obj2->setPosition(3, 0, 0);
	obj2->setScale(1);
	//obj2->setRotation(0, 0, 45);
	addObject(obj2);


	Object* obj3 = new Object(*this, sphereModel, sphereMaterial);
	obj3->setPosition(0, -2, 0.2f);
	//addObject(obj3);


	Model* floorModel = new Model(ntw::getCube());
	setModelProperties(floorModel);
	Material* floorMaterial = new Material();
	floorMaterial->texture = resCache_.loadTexture("tile.bmp");
	floorMaterial->shaderProgram = "standard";

	for(auto i = floorModel->texCoords.begin(); i != floorModel->texCoords.end(); i++)
		*i *= 50;

	Object* floor = new Object(*this, floorModel, floorMaterial);
	floor->setPosition(0, 0, -2);
	floor->setScale(25, 25, 1);
	addObject(floor);

	Object* wall = new Object(*this, floorModel, floorMaterial);
	wall->setPosition(-5, 0, 0);
	wall->setScale(1, 25, 25);
	addObject(wall);


	const int c1 = 1;
	const int c2 = 1;

	for(float x = 0; x < c1; x++){
		for(float y = 0; y < c1; y++){
			for(float z = 0; z < c2; z++){
				PhysicsObject* physObj1 = new PhysicsObject(*this, testModel, boxMaterial, 1);
				//Object* physObj1 = new Object(*this, testModel, testMaterial, RenderType::STATIC, HitboxType::MESH);
				physObj1->setPosition(0 + x - c1 / 2, 4 + y - c1 / 2, z  * 0.75f);
				//physObj1->rotate(0, -30, 0);
				//physObj1->rotate(0, 30, 0);
				//physObj1->setScale(0.5f, 0.5f, 0.1f);
				physObj1->setScale(0.25f);
				addObject(physObj1);
			}
		}
	}

	Object* obj4 = new Object(*this, testModel, testMaterial);
	obj4->setPosition(0, 4, -1);
	obj4->setScale(4, 0.1f, 4);
	obj4->setRotation(-45, 0, 0);
	//addObject(obj4);

	PhysicsObject* physObj2 = new PhysicsObject(*this, testModel, testMaterial, 1);
	physObj2->setPosition(0, 3, 0);
	physObj2->setRotation(90, 30, 0);
	physObj2->setScale(0.3f);
	//addObject(physObj2);

	//((PhysicsObject*)physObj2)->setVelocity(Vec3(0, -3, 0));


	//Object* physObj2 = new Object(testModel, testMaterial, RenderType::DYNAMIC, PhysicsType::SEMI_DYNAMIC, HitboxType::MESH);
	//physObj2->setPosition(0, 3, -0.25f);
	//physObj2->setScale(0.5f);
	//addObject(physObj2);

	Material* blankMaterial = new Material();
	blankMaterial->texture = resCache_.loadTexture("blank.bmp");
	blankMaterial->shaderProgram = "standard";

	vector<Vec3> testPortalPositions = {
		Vec3(1, -2, 0),
		Vec3(-3, -2, 0),
	};

	vector<Quaternion> testPortalRotations = {
		Quaternion().rotate(0, 0, 0),
		Quaternion().rotate(90, 0, 90),
	};

	const float pWidth = 1.5f;
	const float pHeight = 2.1f;
	const float frameScale = 0.05f;

	for(int i = 0; i < testPortalPositions.size(); i ++){

		Vec3 pos = testPortalPositions[i];
		Quaternion rotation = testPortalRotations[i];

		Portal* portal = new Portal(pos, rotation, pWidth, pHeight, i / 4);
		portals_.push_back(portal);

		Vec3 f1 = Vec3((-pWidth / 2) - (frameScale / 2), 0, 0);
		Vec3 f2 = Vec3((pWidth / 2) + (frameScale / 2), 0, 0);
		Vec3 f3 = Vec3(0, 0, (-pHeight / 2) - (frameScale / 2));
		Vec3 f4 = Vec3(0, 0, (pHeight / 2) + (frameScale / 2));

		f1 = Matrix(3, 3, true).rotate(rotation) * f1;
		f2 = Matrix(3, 3, true).rotate(rotation) * f2;
		f3 = Matrix(3, 3, true).rotate(rotation) * f3;
		f4 = Matrix(3, 3, true).rotate(rotation) * f4;

		Object* portalFrame = new Object(*this, testModel, blankMaterial);
		portalFrame->setPosition(pos + f1);
		portalFrame->setScale(frameScale, frameScale, (pHeight / 2) + frameScale);
		portalFrame->setRotation(rotation);
		addObject(portalFrame);

		portalFrame = new Object(*this, testModel, blankMaterial);
		portalFrame->setPosition(pos + f2);
		portalFrame->setScale(frameScale, frameScale, (pHeight / 2) + frameScale);
		portalFrame->setRotation(rotation);
		addObject(portalFrame);

		portalFrame = new Object(*this, testModel, blankMaterial);
		portalFrame->setPosition(pos + f3);
		portalFrame->setScale(pWidth / 2, frameScale, frameScale);
		portalFrame->setRotation(rotation);
		addObject(portalFrame);

		portalFrame = new Object(*this, testModel, blankMaterial);
		portalFrame->setPosition(pos + f4);
		portalFrame->setScale(pWidth / 2, frameScale, frameScale);
		portalFrame->setRotation(rotation);
		addObject(portalFrame);
	}

	// Set portal pairs
	for(int i = 0; i < portals_.size() - 1; i++){

		Portal* p1 = portals_[i];

		for(int j = i + 1; j < portals_.size(); j++){

			Portal* p2 = portals_[j];

			// Matching pair found
			if(p1->getPortalNum() == p2->getPortalNum()){

				// Portals already paired
				if(p1->getPairedPortal() != nullptr || p2->getPairedPortal() != nullptr)
					ntw::warning("Extra portal found with duplicate portal number!");
				else{
					p1->setPairedPortal(p2);
					p2->setPairedPortal(p1);
				}
			}
		}

		// Portal unpaired
		if(p1->getPairedPortal() == nullptr)
			ntw::warning("Unpaired portal found in world!");
	}

	// Initialize player and add to object lists
	Model* playerModel = new Model(ntw::getCube());
	setModelProperties(playerModel);

	player_ = new Player(*this, playerModel, options_.control, window_);
	objects_.push_back(player_);
	physicsObjects_.push_back(player_);


	// Initialize physics engine
	physicsEngine_.init();

	// Update portals and add to physics engine
	for(Portal* p : portals_){
		p->update();
		physicsEngine_.addPortal(p);
	}

	initialized_ = true;
}

void World::update(float timeDelta, bool updatePhysics){
	
	// Update player movement
	player_->updatePlayer(timeDelta, updatePhysics);

	// Update all objects
	for(Object* obj : objects_)
		obj->update(timeDelta);

	// Update physics
	if(updatePhysics)
		physicsEngine_.update();


	// Set sound listener and orientation to player's
	soundEngine_.setListenerPosition(player_->getPosition());
	soundEngine_.setListenerOrientation(player_->getLookVector(), player_->getLookUpVector());

	// Play world sounds, including physics sounds
	//soundEngine_.playWorldSound();

	// Set camera to player orientation after player physics update
	if(updatePhysics){
		camera_.position		= player_->getEyePosition();
		camera_.velocity		= player_->getVelocity();

		camera_.rotationMatrix = player_->getPortalRotation().getTranspose();
	}

	camera_.velocity		= player_->getVelocity();
	camera_.yaw				= player_->getYaw();
	camera_.pitch			= player_->getPitch();

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

	physicsEngine_.cleanup();

	for(Object* object : objects_)
		delete object;

	objects_.clear();
	physicsObjects_.clear();
}


void World::addObject(Object* object){

	objects_.push_back(object);

	// Physics objects
	if(object->getPhysicsType() == PhysicsType::RIGID_BODY || object->getPhysicsType() == PhysicsType::SIMPLE)
		physicsObjects_.push_back((PhysicsObject*)object);


	// Add to renderer
	if(initialized_)
		renderer_.addObject(object);


	// Initialize object in physics engine if necessary
	if(object->getPhysicsType() != PhysicsType::NONE && physicsEngine_.isInitialized())
		physicsEngine_.addObject(object);
}

void World::removeObject(Object* object){

	for(auto i = objects_.begin(); i != objects_.end(); i++){
		if(object == (*i)){
			objects_.erase(i);
			break;
		}
	}

	if(object->getPhysicsType() == PhysicsType::SIMPLE || object->getPhysicsType() == PhysicsType::RIGID_BODY){
		for(auto i = physicsObjects_.begin(); i != physicsObjects_.end(); i++){
			if(object == (*i)){
				physicsObjects_.erase(i);
				break;
			}
		}
	}

	physicsEngine_.removeObject(object);

	delete object;
}


PhysicsEngine& World::getPhysicsEngine(){
	return physicsEngine_;
}

Camera& World::getCamera(){
	return camera_;
}

const vector<Object*>& World::getObjects(){
	return objects_;
}

const vector<Portal*>& World::getPortals(){
	return portals_;
}
