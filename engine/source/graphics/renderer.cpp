#include"renderer.h"

#include"graphics/modelFunc.h"
#include"math/mathFunc.h"
#include"physics/physDefine.h"
#include<math.h>

using ntw::toRadians;


Renderer::Renderer(GraphicsOptions& gOptions) : gOptions_(gOptions) {

}

void Renderer::init(){

	// Load shader programs
	shaderBasic_ = ShaderProgram();
	shaderBasic_.compile("basic.vs", "basic.fs");
	shaderBasic_.link();
	shaderBasic_.use();

	shaderWireframe_ = ShaderProgram();
	shaderWireframe_.compile("wireframe.vs", "wireframe.fs");
	shaderWireframe_.link();

	timeUniformLoc_		= glGetUniformLocation(shaderBasic_.getProgram(), "time");
	viewProjUniformLoc_	= glGetUniformLocation(shaderBasic_.getProgram(), "viewProj");
	viewPosUniformLoc_	= glGetUniformLocation(shaderBasic_.getProgram(), "viewPos");
	modelUniformLoc_	= glGetUniformLocation(shaderBasic_.getProgram(), "model");
}

void Renderer::destroy(){
	shaderBasic_.destroy();
	shaderWireframe_.destroy();
}

void Renderer::initWorldRendering(World* world){

	// Set world
	world_ = world;

	// Batch objects and write data
	vector<Object*>& objects = world->getObjects();
	objectBatches_.clear();

	// Batch static objects according to material
	// Batches for dynamic objects will contain only one object
	for(auto obj = objects.begin(); obj != objects.end(); obj++){

		RenderType type = (*obj)->getRenderType();

		if(type == RenderType::NONE)
			continue;

		bool added = false;

		// For static objects only:
		// If a batch for this texture already exists, add the object to it
		if(type == RenderType::STATIC){
			for(auto batch = objectBatches_.begin(); batch != objectBatches_.end(); batch++){
				if((*batch).renderType == type && (*batch).material == (*obj)->getMaterial()){
					(*batch).objects.push_back(*obj);
					added = true;
					break;
				}
			}
		}

		// If not, create a new batch
		if(!added){
			ObjectBatch b;
			b.renderType = type;
			b.material = (*obj)->getMaterial();
			b.objects.push_back(*obj);

			objectBatches_.push_back(b);
		}
	}


	for(auto batch = objectBatches_.begin(); batch != objectBatches_.end(); batch++){

		// Merge all vertex and texture coordinate data
		vector<float> vertices;
		vector<float> normals;
		vector<float> texCoords;
		int numVertices = 0;

		for(auto obj = (*batch).objects.begin(); obj != (*batch).objects.end(); obj++){
			// For static objects, get temporary model with object transformations applied
			// For dynamic objects, transformations will be applied in the shader, so get standard model
			Model m = (*batch).renderType == RenderType::STATIC ? ntw::getTransformedObjectModel(**obj) : *(*obj)->getModel();

			// Copy data
			std::copy(m.vertices.begin(),	m.vertices.end(),	std::back_inserter(vertices));
			std::copy(m.normals.begin(),	m.normals.end(),	std::back_inserter(normals));
			std::copy(m.texCoords.begin(),	m.texCoords.end(),	std::back_inserter(texCoords));
			numVertices += m.numVertices;
		}


		// Create VAO
		GLuint vao;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);


		// Create vertex buffer
		GLuint vBuffer;
		glGenBuffers(1, &vBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, vBuffer);

		// Write data
		glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(float), vertices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


		// Create normals buffer
		GLuint nmBuffer;
		glGenBuffers(1, &nmBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, nmBuffer);

		// Write data
		glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(float), normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


		// Create texture coordinate buffer
		GLuint tcBuffer;
		glGenBuffers(1, &tcBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, tcBuffer);

		// Write data
		glBufferData(GL_ARRAY_BUFFER, numVertices * 2 * sizeof(float), texCoords.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);


		// Store VAO ID and info
		(*batch).vaoID = vao;
		(*batch).numVertices = numVertices;
		(*batch).bufferIDs = {vBuffer, nmBuffer, tcBuffer};
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Renderer::cleanupWorldRendering(){

	// Delete object VAOs
	for(auto batch = objectBatches_.begin(); batch != objectBatches_.end(); batch++){

		// Delete buffers
		vector<unsigned int>& buffers = (*batch).bufferIDs;

		for(auto j = buffers.begin(); j != buffers.end(); j++)
			glDeleteBuffers(1, (const GLuint*)&(*j));

		glDeleteVertexArrays(1, (const GLuint*)&(*batch).vaoID);
	}

	objectBatches_.clear();
}

void Renderer::renderWorld(int time, float delta){

	delta *= PHYS_DELTA_MULT;

	// World camera
	Camera& camera = world_->getCamera();

	// Interpolate position
	Vec3 camPos = camera.position + camera.velocity * delta;

	// View and projection matrix
	Matrix viewProj;

	// View location, swap y and z axes and negate y
	viewProj.translate(-camPos[0], -camPos[2], camPos[1]);

	// Yaw then pitch rotation
	viewProj.rotate(0, -camera.yaw, 0);
	viewProj.rotate(-camera.pitch, 0, 0);
	viewProj.transpose();

	// Apply projection matrix
	viewProj *= Matrix((float)gOptions_.fov, (float)gOptions_.resolutionX / gOptions_.resolutionY, 0.01f, 1000);

	// Write time and view position uniforms
	shaderBasic_.use();
	glUniform1i(timeUniformLoc_, time);
	glUniformMatrix4fv(viewProjUniformLoc_, 1, GL_FALSE, viewProj.getValuesPtr());
	glUniform3f(viewPosUniformLoc_, camPos[0], camPos[1], camPos[2]);

	// Enable properties
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);

	// Render objects
	for(auto batch = objectBatches_.begin(); batch != objectBatches_.end(); batch++){

		// Add model matrix to MVP matrix if dynamic
		if((*batch).renderType == RenderType::DYNAMIC){
			Object* obj = (*batch).objects[0];

			Vec3 position = obj->getPosition();
			Quaternion rotation = obj->getRotation();

			// Physics interpolation
			if(obj->getPhysicsType() == PhysicsType::DYNAMIC){
				position += ((PhysicsObject*)obj)->getVelocity() * delta;

				Vec3 angularVelocity = ((PhysicsObject*)obj)->getAngularVelocity();
				float angVelMag = angularVelocity.magnitude();
				if(angVelMag != 0)
					rotation.rotate(angularVelocity / angVelMag, angVelMag * delta);
			}

			Matrix model;
			model.scale(obj->getScale());
			model.rotate(rotation);
			model.translate(position);
			model.transpose();

			glUniformMatrix4fv(modelUniformLoc_, 1, GL_FALSE, model.getValuesPtr());
		}

		// For static objects, do not use an additional model matrix
		else
			glUniformMatrix4fv(modelUniformLoc_, 1, GL_FALSE, identity_.getValuesPtr());

		// Bind texture
		glBindTexture(GL_TEXTURE_2D, (*batch).material->texture);
		
		// Select VAO
		glBindVertexArray((*batch).vaoID);

		// Enable buffers
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);

		// Render
		glDrawArrays(GL_TRIANGLES, 0, (*batch).numVertices);

		// Disable buffers
		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);

	// Disable properties
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_MULTISAMPLE);









	// TEMPORARY!
	/*
	vector<ContactManifold> collisions = world_->getPhysicsEngine().getContactManifolds();

	shaderWireframe_.use();
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glEnable(GL_DEPTH_TEST);
	glUniformMatrix4fv(glGetUniformLocation(shaderWireframe_.getProgram(), "viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());

	vector<float> verts;
	int numContacts = 0;

	// For each object
	for(auto i = collisions.begin(); i != collisions.end(); i++){
		for(auto j = (*i).contacts.begin(); j != (*i).contacts.end(); j++){

			Object* obj1 = (*i).objects.object1;
			Object* obj2 = (*i).objects.object2;
			bool obj1Dynamic = obj1->getPhysicsType() == PhysicsType::DYNAMIC;
			bool obj2Dynamic = obj2->getPhysicsType() == PhysicsType::DYNAMIC;

			Matrix obj1Rotation = Matrix(3, 3, true).rotate(
				obj1Dynamic ? ((PhysicsObject*)obj1)->getTRotation() : obj1->getRotation()
			);
			Matrix obj2Rotation = Matrix(3, 3, true).rotate(
				obj2Dynamic ? ((PhysicsObject*)obj2)->getTRotation() : obj2->getRotation()
			);

			// Get updated global contact points
			Vec3 obj1ContactGlobalU = (*j).obj1ContactLocal;
			Vec3 obj2ContactGlobalU = (*j).obj2ContactLocal;

			// Rotate
			obj1ContactGlobalU = obj1Rotation * obj1ContactGlobalU;
			obj2ContactGlobalU = obj2Rotation * obj2ContactGlobalU;

			// Translate
			obj1ContactGlobalU += obj1Dynamic ? ((PhysicsObject*)obj1)->getTPosition() : obj1->getPosition();
			obj2ContactGlobalU += obj2Dynamic ? ((PhysicsObject*)obj2)->getTPosition() : obj2->getPosition();


			verts.push_back((*j).obj1ContactGlobal[0]);
			verts.push_back((*j).obj1ContactGlobal[1]);
			verts.push_back((*j).obj1ContactGlobal[2]);
			numContacts++;
		}
	}
	for(auto i = collisions.begin(); i != collisions.end(); i++){
		for(auto j = (*i).contacts.begin(); j != (*i).contacts.end(); j++){
			verts.push_back((*j).obj2ContactGlobal[0]);
			verts.push_back((*j).obj2ContactGlobal[1]);
			verts.push_back((*j).obj2ContactGlobal[2]);
		}
	}
	for(auto i = collisions.begin(); i != collisions.end(); i++){
		for(auto j = (*i).contacts.begin(); j != (*i).contacts.end(); j++){
			verts.push_back((*j).obj2ContactGlobal[0]);
			verts.push_back((*j).obj2ContactGlobal[1]);
			verts.push_back((*j).obj2ContactGlobal[2]);
			verts.push_back((*j).obj2ContactGlobal[0] + (*j).normal[0]);
			verts.push_back((*j).obj2ContactGlobal[1] + (*j).normal[1]);
			verts.push_back((*j).obj2ContactGlobal[2] + (*j).normal[2]);
		}
	}

	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Create vertex buffer
	GLuint vBuffer;
	glGenBuffers(1, &vBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vBuffer);

	// Write data
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Enable buffers
	glEnableVertexAttribArray(0);

	// Render
	glPointSize(10);
	glUniform3f(glGetUniformLocation(shaderWireframe_.getProgram(), "wireframeColor"), 1, 0, 0);
	glDrawArrays(GL_POINTS, 0, numContacts);

	glUniform3f(glGetUniformLocation(shaderWireframe_.getProgram(), "wireframeColor"), 0, 0, 1);
	glDrawArrays(GL_POINTS, numContacts, numContacts);

	glLineWidth(15);
	glUniform3f(glGetUniformLocation(shaderWireframe_.getProgram(), "wireframeColor"), 0, 1, 0);
	glDrawArrays(GL_LINES, numContacts * 2, numContacts * 2);

	// Disable buffers
	glDisableVertexAttribArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBindVertexArray(0);
	glDisable(GL_DEPTH_TEST);

	// Delete VAO and buffer
	glDeleteBuffers(1, &vBuffer);
	glDeleteVertexArrays(1, &vao);

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	*/
}
