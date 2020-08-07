#include"renderer.h"

#include"objects/modelFunc.h"
#include"math/mathFunc.h"
#include"physics/physDefine.h"
#include<algorithm>
#include<math.h>
#include<limits>

using ntw::toRadians;
using std::min;
using std::max;


void Renderer::initWorldRendering(World* world){

	world_ = world;


	// Initialize main world framebuffer
	fbWorld_ = createFrameBuffer(gOptions_.resolutionX, gOptions_.resolutionY, false, true, false, gOptions_.msaaSamples);

	// Secondary framebuffer for portal rendering
	if(!world->getPortals().empty())
		fbWorldPortal_ = createFrameBuffer(gOptions_.resolutionX, gOptions_.resolutionY, false, true, true, gOptions_.msaaSamples);


	// Batch objects and write data
	const vector<Object*>& objects = world->getObjects();
	objectBatchGroups_.clear();

	// Batch static objects according to material
	// Batches for dynamic objects will contain only one object
	for(Object* object : objects)
		addObjectToBatchGroups(object);


	// Initialize static batches
	for(BatchGroup& group : objectBatchGroups_)
		for(ObjectBatch& batch : group.batches)
			if(batch.renderType == RenderType::STATIC)
				initObjectBatch(batch);


	// Batch portals
	const vector<Portal*>& portals = world_->getPortals();

	for(Portal* p : portals)
		addPortalBatch(p);


	// Initialize skybox
	vector<float> cubeVerts = ntw::getCube().vertices;

	// Create VAO
	glGenVertexArrays(1, &skyboxVao_);
	glBindVertexArray(skyboxVao_);

	// Create vertex buffer
	glGenBuffers(1, &skyboxVBuffer_);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBuffer_);

	// Write data
	glBufferData(GL_ARRAY_BUFFER, cubeVerts.size() * sizeof(float), cubeVerts.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Renderer::addObjectToBatchGroups(Object* object){

	if(object->getRenderType() == RenderType::NONE)
		return;

	// Check if a batch group with this object's shader already exists
	for(BatchGroup& group : objectBatchGroups_){
		if(group.shaderProgram == object->getMaterial()->shaderProgram){

			// For static objects only:
			// If a batch for this material already exists, add the object to it
			if(object->getRenderType() == RenderType::STATIC){
				for(ObjectBatch& batch : group.batches){
					if(batch.renderType == RenderType::STATIC && batch.material == object->getMaterial()){
						batch.objects.push_back(object);
						return;
					}
				}
			}

			// If not, create a new batch
			ObjectBatch& batch = addObjectBatch(group, object);

			// Initialize if not static
			if(batch.renderType != RenderType::STATIC)
				initObjectBatch(batch);
		}
	}

	// No batch group found for this shader, create one
	BatchGroup group;
	group.shaderProgram = object->getMaterial()->shaderProgram;
	objectBatchGroups_.push_back(group);

	ObjectBatch& batch = addObjectBatch(objectBatchGroups_[objectBatchGroups_.size() - 1], object);

	if(batch.renderType != RenderType::STATIC)
		initObjectBatch(batch);
}

Renderer::ObjectBatch& Renderer::addObjectBatch(BatchGroup& group, Object* object){

	ObjectBatch b;
	b.renderType = object->getRenderType();
	b.material = object->getMaterial();
	b.objects.push_back(object);

	group.batches.push_back(b);
	return group.batches[group.batches.size() - 1];
}

void Renderer::initObjectBatch(ObjectBatch& batch){

	// Merge all vertex and texture coordinate data
	vector<float> vertices;
	vector<float> normals;
	vector<float> texCoords;
	int numVertices = 0;

	for(Object* obj : batch.objects){
		// For static objects, get temporary model with object transformations applied
		// For dynamic objects, transformations will be applied in the shader, so get standard model
		Model m = batch.renderType == RenderType::STATIC ? ntw::getTransformedObjectModel(*obj) : *(obj)->getModel();

		// Copy data
		std::copy(m.vertices.begin(), m.vertices.end(), std::back_inserter(vertices));
		std::copy(m.normals.begin(), m.normals.end(), std::back_inserter(normals));
		std::copy(m.texCoords.begin(), m.texCoords.end(), std::back_inserter(texCoords));
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
	batch.vaoId = vao;
	batch.numVertices = numVertices;
	batch.bufferIds = {vBuffer, nmBuffer, tcBuffer};
}

void Renderer::addObject(Object* object){

	// For objects added after initialization

	// Dynamic objects only
	if(object->getRenderType() != RenderType::DYNAMIC){
		if(object->getRenderType() == RenderType::STATIC)
			ntw::warning("Static object added after renderer initialization, it will not be rendered");

		return;
	}

	// Create and initialize object batch
	addObjectToBatchGroups(object);
}

void Renderer::addPortalBatch(Portal* portal){

	// Render portal as a cube
	vector<float> cubeVerts = ntw::getCube().vertices;
	vector<float> vertices;

	// Portal transformations
	Vec3 size = Vec3(portal->getWidth() / 2, NTW_NEAR_CLIP * 2, portal->getHeight() / 2);
	Matrix rotation = Matrix(3, 3, true).rotate(portal->getRotation());


	// Add vertices
	for(int i = 0; i < cubeVerts.size(); i += 3){

		// Current vertex
		Vec3 v = Vec3(cubeVerts[i], cubeVerts[i + 1], cubeVerts[i + 2]);

		v *= size;
		v = rotation * v;

		// Adjust size to fix z-fighting
		v *= 0.999f;

		v += portal->getPosition();

		vertices.push_back(v[0]);
		vertices.push_back(v[1]);
		vertices.push_back(v[2]);
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
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


	// Store VAO ID and info
	PortalBatch batch;
	batch.portal = portal;
	batch.vaoId = vao;
	batch.vBufferId = vBuffer;

	portalBatches_.push_back(batch);
}


void Renderer::cleanupWorldRendering(){

	// Delete object VAOs
	for(BatchGroup& group : objectBatchGroups_){
		for(ObjectBatch& batch : group.batches){

			// Delete buffers
			for(GLuint buffer : batch.bufferIds)
				glDeleteBuffers(1, &buffer);

			glDeleteVertexArrays(1, &batch.vaoId);
		}
	}

	objectBatchGroups_.clear();


	// Delete portal VAOs
	for(PortalBatch& batch : portalBatches_){
		glDeleteVertexArrays(1, &batch.vaoId);
		glDeleteBuffers(1, &batch.vBufferId);
	}

	portalBatches_.clear();


	// Delete skybox VAO
	glDeleteBuffers(1, &skyboxVao_);
	glDeleteVertexArrays(1, &skyboxVBuffer_);


	// Delete framebuffers
	glDeleteTextures(1, &fbWorld_.textureId);
	glDeleteTextures(1, &fbWorldPortal_.textureId);

	glDeleteRenderbuffers(1, &fbWorld_.depthBufferId);
	glDeleteRenderbuffers(1, &fbWorld_.depthBufferId);

	glDeleteFramebuffers(1, &fbWorld_.id);
	glDeleteFramebuffers(1, &fbWorldPortal_.id);
}

void Renderer::setViewProj(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly){

	setViewProjSub(camera, viewProj, viewProjRotOnly);

	// Apply projection matrix
	Matrix proj = Matrix::projectionMatrix((float)gOptions_.fov, (float)gOptions_.resolutionX / gOptions_.resolutionY, NTW_NEAR_CLIP, NTW_FAR_CLIP);

	viewProj *= proj;
	viewProjRotOnly *= proj;
}

void Renderer::setViewProj(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly, Vec3 clipPlaneNormal, float clipPlaneDistance){

	setViewProjSub(camera, viewProj, viewProjRotOnly);

	// Apply projection matrix with oblique near clipping plane
	Matrix proj = Matrix::projectionMatrix((float)gOptions_.fov, (float)gOptions_.resolutionX / gOptions_.resolutionY, NTW_FAR_CLIP, clipPlaneNormal, clipPlaneDistance);

	viewProj *= proj;
	viewProjRotOnly *= proj;
}

void Renderer::setViewProjSub(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly){

	// View location, swap y and z axes and negate y
	viewProj.translate(-camera.position[0], -camera.position[1], -camera.position[2]);

	// Initial orientation
	viewProj = camera.rotationMatrix * viewProj;

	// Rotation
	viewProj.rotate(-90, 90 - camera.yaw, 0);
	viewProj.rotate(-camera.pitch, 0, 0);

	viewProj.transpose();

	// Matrix without translation
	viewProjRotOnly = viewProj;
	viewProjRotOnly.set(3, 0, 0);
	viewProjRotOnly.set(3, 1, 0);
	viewProjRotOnly.set(3, 2, 0);
}

void Renderer::renderWorld(int time, float physTimeDelta){

	// Bind world framebuffer and render
	glBindFramebuffer(GL_FRAMEBUFFER, fbWorld_.id);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Get camera and apply physics interpolation
	Camera camera = world_->getCamera();
	camera.position += camera.velocity * physTimeDelta;

	// Check if interpolation has moved camera past portal
	auto portalCollisions = world_->getPhysicsEngine().getPortalCollisions();

	for(auto& i : portalCollisions){
		PortalCollisionInfo& info = i.second;

		// Only portal collisions with player affect camera
		if(info.withPlayer){
			Portal* portal = info.objectPortalPair.portal;
			bool cameraInFront = portal->isPointInFront(camera.position);

			// Camera moved past portal, teleport it and add rotation matrix
			if(info.objectInFront ^ cameraInFront){
				camera.position = portal->getTransformedVector(camera.position);
				camera.rotationMatrix = portal->getRotationMatrix().getTranspose() * camera.rotationMatrix;
			}
		}
	}


	Matrix viewProj;
	Matrix viewProjRotOnly;
	setViewProj(camera, viewProj, viewProjRotOnly);

	renderWorldSub(camera, viewProj, viewProjRotOnly, time, physTimeDelta);

	// Check portal visibility
	vector<PortalBatch> visiblePortalBatches;
	//Matrix viewProjTranspose = viewProj.getTranspose();

	for(const PortalBatch& batch : portalBatches_){

		// Distance check to prevent clipping when moving in and out of portal
		if(abs((batch.portal->getPosition() - camera.position) * batch.portal->getNormal()) < NTW_NEAR_CLIP * 2){
			visiblePortalBatches.push_back(batch);
			continue;
		}

		const vector<Vec3> verts = batch.portal->getVertices();

		// Bounds for secondary check
		Vec3 lowerBound = Vec3(std::numeric_limits<float>::max());
		Vec3 upperBound = -lowerBound;

		// Project each of the portal's vertices and check if it is visible
		for(const Vec3& vertex : verts){

			// Get vertex and apply projection
			Matrix v = Matrix(1, 4);
			v.set(0, 0, vertex[0]);
			v.set(0, 1, vertex[1]);
			v.set(0, 2, vertex[2]);
			v.set(0, 3, 1);
			v *= viewProj;

			// Normalize
			float w = v.get(0, 3);
			float xw = v.get(0, 0) / w;
			float yw = v.get(0, 1) / w;
			float zw = v.get(0, 2) / w;

			// Check vertex visibility
			if(	xw >= -1 && xw <= 1 &&
				yw >= -1 && yw <= 1 &&
				zw >= -1 && zw <= 1){

				// Visible, add portal to list
				visiblePortalBatches.push_back(batch);
				goto portalLoop;
			}
			
			// Set bounds for secondary check
			lowerBound[0] = min(lowerBound[0], xw);
			lowerBound[1] = min(lowerBound[1], yw);
			lowerBound[2] = min(lowerBound[2], zw);

			upperBound[0] = max(upperBound[0], xw);
			upperBound[1] = max(upperBound[1], yw);
			upperBound[2] = max(upperBound[2], zw);
		}

		// Secondary checks for when vertices are out of view
		for(int i = 0; i < 3; i++)
			if(lowerBound[i] > 1 || upperBound[i] < -1)
				goto portalLoop;

		// Visible
		visiblePortalBatches.push_back(batch);

	portalLoop:;
	}

	// Render portals
	ShaderProgram& portalShader = shaderPrograms_["portal"];

	for(const PortalBatch& batch : visiblePortalBatches){

		// Get portals
		Portal* portal = batch.portal;
		Portal* pairedPortal = portal->getPairedPortal();

		// Set camera position/rotation
		Camera portalCamera = camera;
		portalCamera.position = portal->getTransformedVector(camera.position);
		portalCamera.rotationMatrix = portal->getRotationMatrix().getTranspose() * camera.rotationMatrix;


		// Get viewProj matrix with oblique near clipping plane
		Vec3 planeNormal = portal->getNormal();

		// Rotate portal plane into camera space
		Matrix cameraRotation = camera.rotationMatrix;
		cameraRotation.rotate(-90, 90 - camera.yaw, 0);
		cameraRotation.rotate(-camera.pitch, 0, 0);
		//cameraRotation.rotate(0, 0, -portalCamera.roll);

		planeNormal = cameraRotation * planeNormal;

		float distance = abs((portal->getPosition() - camera.position) * portal->getNormal());
		distance -= NTW_NEAR_CLIP * 2;
		distance = distance < NTW_NEAR_CLIP * 2 ? NTW_NEAR_CLIP * 2 : distance;

		// Get clipped viewProj matrix
		Matrix viewProjPortal;
		Matrix viewProjRotOnlyPortal;
		setViewProj(portalCamera, viewProjPortal, viewProjRotOnlyPortal, -planeNormal, -distance);


		// Clear portal framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, fbWorldPortal_.id);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Blit world framebuffer to portal framebuffer
		// This is so clipped areas are rendered as the normal world
		// This fixes rendering glitches while walking through/standing in portals
		glEnable(GL_MULTISAMPLE);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, fbWorld_.id);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbWorldPortal_.id);
		glBlitFramebuffer(0, 0, gOptions_.resolutionX, gOptions_.resolutionY, 0, 0, gOptions_.resolutionX, gOptions_.resolutionY, GL_COLOR_BUFFER_BIT, GL_LINEAR);

		glDisable(GL_MULTISAMPLE);
		

		// Render world from pair portal perspective to portal framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, fbWorldPortal_.id);
		renderWorldSub(portalCamera, viewProjPortal, viewProjRotOnlyPortal, time, physTimeDelta);

		glBindFramebuffer(GL_FRAMEBUFFER, fbWorld_.id);


		// Shader uniforms
		portalShader.use();
		glUniform1i(portalShader.getUniformLocation("time"), time);
		glUniformMatrix4fv(portalShader.getUniformLocation("viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());

		// Bind texture
		glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, fbWorldPortal_.textureId);

		glEnable(GL_DEPTH_TEST);
		glEnable(GL_MULTISAMPLE);

		// Select VAO
		glBindVertexArray(batch.vaoId);

		// Render
		glEnableVertexAttribArray(0);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glDisableVertexAttribArray(0);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_MULTISAMPLE);
	}

	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::renderWorldSub(const Camera& camera, const Matrix& viewProj, const Matrix& viewProjRotOnly, int time, float physTimeDelta){

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);


	// Loop through batch groups
	for(BatchGroup& group : objectBatchGroups_){

		// Check that shader is present
		if(shaderPrograms_.find(group.shaderProgram) == shaderPrograms_.end()){
			ntw::error("Shader program \"" + group.shaderProgram + "\" is not loaded!");
			continue;
		}

		// Get shader program to use
		ShaderProgram& shader = shaderPrograms_[group.shaderProgram];
		shader.use();

		// Write time and view position uniforms for current shader
		glUniform1i(shader.getUniformLocation("time"), time);
		glUniformMatrix4fv(shader.getUniformLocation("viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());
		glUniform3f(shader.getUniformLocation("viewPos"), camera.position[0], camera.position[1], camera.position[2]);


		// Render objects
		for(ObjectBatch& batch : group.batches){

			// Add model matrix to MVP matrix if dynamic
			if(batch.renderType == RenderType::DYNAMIC){
				Object* obj = batch.objects[0];

				Vec3 position = obj->getPosition();
				Quaternion rotation = obj->getRotation();

				// Rigid body physics interpolation
				if(obj->getPhysicsType() == PhysicsType::SIMPLE || obj->getPhysicsType() == PhysicsType::RIGID_BODY){
					position += ((PhysicsObject*)obj)->getVelocity() * physTimeDelta;

					Vec3 angularVelocity = ((PhysicsObject*)obj)->getAngularVelocity();
					float angVelMag = angularVelocity.magnitude();
					if(angVelMag != 0)
						rotation.rotate(angularVelocity, angVelMag * physTimeDelta);
				}

				Matrix model;
				model.scale(obj->getScale());
				model.rotate(rotation);
				model.translate(position);
				model.transpose();

				glUniformMatrix4fv(shader.getUniformLocation("model"), 1, GL_FALSE, model.getValuesPtr());
			}

			// For static objects, do not use an additional model matrix
			else
				glUniformMatrix4fv(shader.getUniformLocation("model"), 1, GL_FALSE, identity_.getValuesPtr());

			// Rebind texture if necessary
			GLint texture;
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);

			if(texture != batch.material->texture)
				glBindTexture(GL_TEXTURE_2D, batch.material->texture);


			// Select VAO
			glBindVertexArray(batch.vaoId);

			// Enable buffers
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			glEnableVertexAttribArray(2);

			// Render
			glDrawArrays(GL_TRIANGLES, 0, batch.numVertices);

			// Disable buffers
			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);
			glDisableVertexAttribArray(2);
		}
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);

	glDisable(GL_CULL_FACE);


	// Render skybox
	ShaderProgram& shader = shaderPrograms_["skybox"];
	shader.use();

	glUniform1i(shader.getUniformLocation("time"), time);
	glUniformMatrix4fv(shader.getUniformLocation("viewProj"), 1, GL_FALSE, viewProjRotOnly.getValuesPtr());

	glDepthFunc(GL_LEQUAL);

	glBindVertexArray(skyboxVao_);

	glEnableVertexAttribArray(0);
	glDrawArrays(GL_TRIANGLES, 0, 36);

	glDisableVertexAttribArray(0);
	glBindVertexArray(0);


	glDisable(GL_DEPTH_TEST);
	glDisable(GL_MULTISAMPLE);




	// TEMPORARY!
	/*
	vector<ContactManifold> collisions = world_->getPhysicsEngine().getContactManifolds();

	shaderPrograms_["wireframe"].use();
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glEnable(GL_DEPTH_TEST);
	glUniformMatrix4fv(shaderPrograms_["wireframe"].getUniformLocation("viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());

	vector<float> verts;
	int numContacts = 0;

	// For each object
	for(auto i = collisions.begin(); i != collisions.end(); i++){
		for(auto j = (*i).contacts.begin(); j != (*i).contacts.end(); j++){
			//verts.push_back((*i).objects.object1->getPosition()[0] + (*j).obj1ContactVector[0]);
			//verts.push_back((*i).objects.object1->getPosition()[1] + (*j).obj1ContactVector[1]);
			//verts.push_back((*i).objects.object1->getPosition()[2] + (*j).obj1ContactVector[2]);
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
			verts.push_back((*j).obj2ContactGlobal[0] + (*j).normal[0] / 10);
			verts.push_back((*j).obj2ContactGlobal[1] + (*j).normal[1] / 10);
			verts.push_back((*j).obj2ContactGlobal[2] + (*j).normal[2] / 10);
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
	glPointSize(20);
	glUniform3f(shaderPrograms_["wireframe"].getUniformLocation("wireframeColor"), 1, 0, 0);
	glDrawArrays(GL_POINTS, 0, numContacts);

	glUniform3f(shaderPrograms_["wireframe"].getUniformLocation("wireframeColor"), 0, 0, 1);
	glDrawArrays(GL_POINTS, numContacts, numContacts);

	glLineWidth(15);
	glUniform3f(shaderPrograms_["wireframe"].getUniformLocation("wireframeColor"), 0, 1, 0);
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
