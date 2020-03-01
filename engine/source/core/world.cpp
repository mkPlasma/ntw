#include"world.h"

#include"graphics/modelFunc.h"


World::World(Options& options, TextureCache& texCache, Window& window) : options_(options), texCache_(texCache),
	physicsEngine_(objects_, physicsObjects_), player_(new Player(options_.control, window)) {

}

void World::test(){

	// Texture
	Texture* tex = texCache_.loadTexture("star3.bmp");
	textures_.push_back(tex);

	Model* testModel = new Model(ntw::getCube(false));
	testModel->texture = tex;

	/*
	const int s = 32;

	for(int x = -s / 2; x < s / 2; x++){
		for(int y = -s / 2; y < s / 2; y++){
			for(int z = -s / 2; z < s / 2; z++){
				Object* object = new Object(testModel, RENDER_STATIC, HITBOX_CUBE);
				object->setPosition(x * 2, y * 2, z * 2);
				object->setScale(0.2);
				objects_.push_back(object);
			}
		}
	}
	*/

	Object* obj1 = new Object(testModel, RENDER_STATIC, HITBOX_CUBE);
	obj1->setPosition(-3, 0, 0);
	obj1->setScale(0.5);
	obj1->setRotation(15, 15, 15);
	objects_.push_back(obj1);

	Object* obj2 = new Object(testModel, RENDER_STATIC, HITBOX_CUBE);
	obj2->setPosition(Vec3(3, 0, 0));
	objects_.push_back(obj2);


	Texture* sphereTex = texCache_.loadTexture("earth.bmp");
	textures_.push_back(sphereTex);

	Model* sphereModel = new Model(ntw::getSphere(16, 16, true));
	sphereModel->texture = sphereTex;

	Object* obj3 = new Object(sphereModel, RENDER_STATIC, HITBOX_MESH);
	obj3->setPosition(0, -2, 0.2f);
	objects_.push_back(obj3);

	Texture* floorTex = texCache_.loadTexture("tile.bmp");
	textures_.push_back(floorTex);

	Model* floorModel = new Model(ntw::getCube());
	floorModel->texture = floorTex;

	for(auto i = floorModel->texCoords.begin(); i != floorModel->texCoords.end(); i++)
		*i *= 50;

	Object* floor = new Object(floorModel, RENDER_STATIC, HITBOX_CUBE);
	floor->setPosition(0, 0, -2);
	floor->setScale(25, 25, 1);
	objects_.push_back(floor);


	const int c1 = 1;
	const int c2 = 1;

	for(float x = 0; x < c1; x++){
		for(float y = 0; y < c1; y++){
			for(float z = 0; z < c2; z++){
				PhysicsObject* physObj1 = new PhysicsObject(testModel, HITBOX_CUBE, 1);
				physObj1->setPosition(0 + x - c1 / 2, 4 + y - c1 / 2, 0 + z  * 0.51f);
				//physObj1->rotate(45, 0, 0);
				//physObj1->rotate(0, 30, 0);
				//physObj1->setScale(0.5f, 0.5f, 0.1f);
				physObj1->setScale(0.25f);
				physObj1->init();
				objects_.push_back(physObj1);
				physicsObjects_.push_back(physObj1);
			}
		}
	}

	Object* physObj2 = new Object(testModel, RENDER_DYNAMIC, PHYS_SEMI_DYNAMIC, HITBOX_CUBE);
	physObj2->setPosition(0, 3, -0.25f);
	physObj2->setScale(0.5f);
	//objects_.push_back(physObj2);

	// Initialize player and add to object lists
	player_->initPlayer();
	player_->init();

	objects_.push_back(player_);
	physicsObjects_.push_back(player_);

	// Initialize physics engine
	physicsEngine_.init();
}

void World::update(const bool& updatePhysics){
	
	// Update player movement
	player_->updatePlayer(updatePhysics);

	// Update physics on required frames
	if(updatePhysics)
		physicsEngine_.update();

	// Set camera to player orientation after player physics update
	camera_.position	= player_->getPosition();
	camera_.velocity	= player_->getVelocity();
	camera_.yaw			= player_->getYaw();
	camera_.pitch		= player_->getPitch();
}

void World::unload(){

	// Unload textures
	for(auto i = textures_.begin(); i != textures_.end(); i++)
		texCache_.unloadTexture(*i);

	textures_.clear();

	// Delete objects
	// TODO: TEMPORARY - NEED TO CACHE MODEL GEOMETRY
	for(auto i = objects_.begin(); i != objects_.end(); i++){
		Model* m = (*i)->getModel();
		delete *i;
	}

	objects_.clear();
}


Camera& World::getCamera(){
	return camera_;
}

vector<Object*>& World::getObjects(){
	return objects_;
}

vector<Texture*>& World::getTextures(){
	return textures_;
}

PhysicsEngine& World::getPhysicsEngine(){
	return physicsEngine_;
}
