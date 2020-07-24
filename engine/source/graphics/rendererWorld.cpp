#include"renderer.h"

#include"objects/modelFunc.h"
#include"math/mathFunc.h"
#include"physics/physDefine.h"
#include<math.h>

using ntw::toRadians;


void Renderer::initWorldRendering(World* world){

	world_ = world;


	// Initialize main world framebuffer
	fbWorld_ = createFrameBuffer(gOptions_.resolutionX, gOptions_.resolutionY, false, true, gOptions_.msaaSamples);

	// Secondary framebuffer for portal rendering
	if(!world->getPortals().empty())
		fbWorldPortal_ = createFrameBuffer(gOptions_.resolutionX, gOptions_.resolutionY, false, true, gOptions_.msaaSamples);


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

void Renderer::addPortalBatch(Portal* p){

	// Get vertex data
	const vector<Vec3>& portalVerts = p->getVerts();
	vector<float> vertices;

	// Add portal vertices as two triangles
	vertices.push_back(portalVerts[0][0]);
	vertices.push_back(portalVerts[0][1]);
	vertices.push_back(portalVerts[0][2]);

	vertices.push_back(portalVerts[1][0]);
	vertices.push_back(portalVerts[1][1]);
	vertices.push_back(portalVerts[1][2]);

	vertices.push_back(portalVerts[2][0]);
	vertices.push_back(portalVerts[2][1]);
	vertices.push_back(portalVerts[2][2]);


	vertices.push_back(portalVerts[1][0]);
	vertices.push_back(portalVerts[1][1]);
	vertices.push_back(portalVerts[1][2]);

	vertices.push_back(portalVerts[2][0]);
	vertices.push_back(portalVerts[2][1]);
	vertices.push_back(portalVerts[2][2]);

	vertices.push_back(portalVerts[3][0]);
	vertices.push_back(portalVerts[3][1]);
	vertices.push_back(portalVerts[3][2]);


	// Create VAO
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);


	// Create vertex buffer
	GLuint vBuffer;
	glGenBuffers(1, &vBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vBuffer);

	// Write data
	glBufferData(GL_ARRAY_BUFFER, 18 * sizeof(float), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


	// Create normals buffer
	/*
	GLuint nmBuffer;
	glGenBuffers(1, &nmBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, nmBuffer);

	// Write data
	glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(float), normals.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	*/

	// Store VAO ID and info
	PortalBatch batch;
	batch.portal = p;
	batch.vaoId = vao;
	batch.bufferIds = {vBuffer/*, nmBuffer*/};

	portalBatches_.push_back(batch);
}


void Renderer::cleanupWorldRendering(){

	// Delete object VAOs
	for(BatchGroup& group : objectBatchGroups_){
		for(ObjectBatch& batch : group.batches){

			// Delete buffers
			vector<unsigned int>& buffers = batch.bufferIds;

			for(auto j = buffers.begin(); j != buffers.end(); j++)
				glDeleteBuffers(1, (const GLuint*)&(*j));

			glDeleteVertexArrays(1, (const GLuint*)&batch.vaoId);
		}
	}

	objectBatchGroups_.clear();


	// Delete portal VAOs
	for(PortalBatch& batch : portalBatches_){

		// Delete buffers
		vector<unsigned int>& buffers = batch.bufferIds;

		for(auto j = buffers.begin(); j != buffers.end(); j++)
			glDeleteBuffers(1, (const GLuint*)&(*j));

		glDeleteVertexArrays(1, (const GLuint*)&batch.vaoId);
	}

	portalBatches_.clear();


	// Delete framebuffers
	glDeleteTextures(1, &fbWorld_.textureId);
	glDeleteTextures(1, &fbWorldPortal_.textureId);

	glDeleteRenderbuffers(1, &fbWorld_.depthBufferId);
	glDeleteRenderbuffers(1, &fbWorld_.depthBufferId);

	glDeleteFramebuffers(1, &fbWorld_.id);
	glDeleteFramebuffers(1, &fbWorldPortal_.id);
}

Matrix Renderer::getViewProj(const Camera& camera){

	Matrix viewProj = getViewProjSub(camera);

	// Apply projection matrix
	viewProj *= Matrix::projectionMatrix((float)gOptions_.fov, (float)gOptions_.resolutionX / gOptions_.resolutionY, NTW_NEAR_CLIP, NTW_FAR_CLIP);

	return viewProj;
}

Matrix Renderer::getViewProj(const Camera& camera, Vec3 clipPlaneNormal, float clipPlaneDistance){

	Matrix viewProj = getViewProjSub(camera);

	// Apply projection matrix with oblique near clipping plane
	viewProj *= Matrix::projectionMatrix((float)gOptions_.fov, (float)gOptions_.resolutionX / gOptions_.resolutionY, NTW_FAR_CLIP, clipPlaneNormal, clipPlaneDistance);

	return viewProj;
}

Matrix Renderer::getViewProjSub(const Camera& camera){

	Matrix viewProj;

	// View location, swap y and z axes and negate y
	viewProj.translate(-camera.position[0], -camera.position[2], camera.position[1]);

	// Yaw then pitch rotation
	viewProj.rotate(0, 90 - camera.yaw, 0);
	viewProj.rotate(-camera.pitch, 0, 0);
	viewProj.rotate(0, 0, -camera.roll);
	viewProj.transpose();

	return viewProj;
}

#include<functional>
#include<iostream>
using std::cout;
using std::endl;

void Renderer::renderWorld(int time, float physTimeDelta){

	// Bind world framebuffer and render
	glBindFramebuffer(GL_FRAMEBUFFER, fbWorld_.id);

	const Camera& camera = world_->getCamera();
	Matrix viewProj = getViewProj(camera);
	renderWorldSub(camera, viewProj, time, physTimeDelta);

	// Check portal visibility
	vector<PortalBatch> visiblePortalBatches;
	//Matrix viewProjTranspose = viewProj.getTranspose();

	for(const PortalBatch& batch : portalBatches_){
		visiblePortalBatches.push_back(batch);
		/*
		const vector<Vec3>& verts = batch.portal->getVerts();

		// Project each of the portal's vertices and check if it is visible
		for(const Vec3& vertex : verts){

			// Get vertex and apply projection
			Matrix v = Matrix(4, 1);
			v.set(0, 0, vertex[0]);
			v.set(1, 0, vertex[1]);
			v.set(2, 0, vertex[2]);
			v.set(3, 0, 1);
			v = viewProj * v;

			// Normalize
			float w = v.get(3, 0);
			float xw = v.get(0, 0) / w;
			float yw = v.get(1, 0) / w;
			float zw = v.get(2, 0) / w;
			cout << xw << "\t" << yw << "\t" << zw << endl;

			// Visible, add portal to list
			if(	xw > -1 && xw < 1 &&
				yw > -1 && yw < 1 &&
				zw > -1 && zw < 1){

				visiblePortalBatches.push_back(batch);
				break;
			}
		}

		//cout << endl;
		*/
	}

	// Render portals
	ShaderProgram& shader = shaderPrograms_["portal"];

	for(const PortalBatch& batch : visiblePortalBatches){

		// Get portals
		Portal* portal = batch.portal;
		Portal* pairedPortal = portal->getPairedPortal();


		// Angle difference between two portals
		Matrix portalRotation = Matrix(3, 3, true).rotate(pairedPortal->getRotation() - portal->getRotation());

		Camera portalCamera = camera;

		// Set camera position
		Vec3 cameraVector = portalCamera.position - portal->getPosition();
		cameraVector = portalRotation * cameraVector;

		portalCamera.position = pairedPortal->getPosition() - cameraVector;
		portalCamera.position[2] = -portalCamera.position[2];


		// Set camera angle
		Vec3 cameraDirectionVector = -Vec3(camera.yaw, camera.pitch);

		// Rotate by the difference between the portals' rotations
		cameraDirectionVector = portalRotation * cameraDirectionVector;

		float pitch = -asinf(cameraDirectionVector[2]);
		float yaw = atan2f(cameraDirectionVector[1], cameraDirectionVector[0]);

		portalCamera.pitch	= ntw::toDegrees(pitch);
		//portalCamera.roll	= cameraAngle[1];
		portalCamera.yaw	= ntw::toDegrees(yaw);


		// Get viewProj matrix with oblique near clipping plane
		Vec3 normal = pairedPortal->getNormal();
		Vec3 planeNormal	= Matrix(3, 3, true).rotate(Vec3(0, atan2f(normal[1], normal[0]) - yaw + ntw::toRadians(180), 0), false) * Vec3(0, 0, -1);
		planeNormal			= Matrix(3, 3, true).rotate(Vec3(-asinf(normal[2]) - pitch, 0, 0), false) * planeNormal;

		float distance = (pairedPortal->getPosition() - portalCamera.position) * normal;

		Matrix viewProjPortal = getViewProj(portalCamera, planeNormal, distance);


		// Render world from pair portal perspective to portal framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, fbWorldPortal_.id);
		renderWorldSub(portalCamera, viewProjPortal, time, physTimeDelta);

		glBindFramebuffer(GL_FRAMEBUFFER, fbWorld_.id);

		// Bind texture
		glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, fbWorldPortal_.textureId);


		shader.use();
		glUniform1i(shader.getUniformLocation("time"), time);
		glUniformMatrix4fv(shader.getUniformLocation("viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());
		glUniform2i(shader.getUniformLocation("screenSize"), gOptions_.resolutionX, gOptions_.resolutionY);

		glEnable(GL_DEPTH_TEST);
		glEnable(GL_MULTISAMPLE);


		// Select VAO
		glBindVertexArray(batch.vaoId);

		// Enable buffers
		glEnableVertexAttribArray(0);
		//glEnableVertexAttribArray(1);

		// Render
		glDrawArrays(GL_TRIANGLES, 0, 6);

		// Disable buffers
		glDisableVertexAttribArray(0);
		//glDisableVertexAttribArray(1);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_MULTISAMPLE);
	}

	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::renderWorldSub(const Camera& camera, int time, float physTimeDelta){
	Matrix viewProj = getViewProj(camera);
	renderWorldSub(camera, viewProj, time, physTimeDelta);
}

void Renderer::renderWorldSub(const Camera& camera, const Matrix& viewProj, int time, float physTimeDelta){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Enable properties
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
				if(obj->getPhysicsType() == PhysicsType::DYNAMIC){
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

	// Disable properties
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_MULTISAMPLE);





	AABBTree::Node* root = world_->getPhysicsEngine().getRoot();

	shaderPrograms_["wireframe"].use();
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glEnable(GL_DEPTH_TEST);
	glUniformMatrix4fv(shaderPrograms_["wireframe"].getUniformLocation("viewProj"), 1, GL_FALSE, viewProj.getValuesPtr());

	vector<float> verts;
	vector<float> verts2;

	std::function<void(AABBTree::Node*)> l_addNode = [&](AABBTree::Node* n){
		if(n->isLeaf()){
			float lx = n->aabb.lowerBound[0];
			float ly = n->aabb.lowerBound[1];
			float lz = n->aabb.lowerBound[2];
			float ux = n->aabb.upperBound[0];
			float uy = n->aabb.upperBound[1];
			float uz = n->aabb.upperBound[2];

			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(lz);
			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(lz);

			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(lz);
			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(lz);

			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(uz);
			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(uz);

			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(uz);
			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(uz);


			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(lz);
			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(lz);

			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(lz);
			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(lz);

			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(uz);
			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(uz);

			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(uz);
			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(uz);


			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(lz);
			verts.push_back(lx);	verts.push_back(ly);	verts.push_back(uz);

			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(lz);
			verts.push_back(ux);	verts.push_back(ly);	verts.push_back(uz);

			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(lz);
			verts.push_back(lx);	verts.push_back(uy);	verts.push_back(uz);

			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(lz);
			verts.push_back(ux);	verts.push_back(uy);	verts.push_back(uz);
		}
		else{
			float lx = n->aabb.lowerBound[0];
			float ly = n->aabb.lowerBound[1];
			float lz = n->aabb.lowerBound[2];
			float ux = n->aabb.upperBound[0];
			float uy = n->aabb.upperBound[1];
			float uz = n->aabb.upperBound[2];

			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(lz);
			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(lz);

			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(lz);
			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(lz);

			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(uz);
			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(uz);

			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(uz);
			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(uz);


			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(lz);
			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(lz);

			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(lz);
			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(lz);

			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(uz);
			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(uz);

			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(uz);
			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(uz);


			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(lz);
			verts2.push_back(lx);	verts2.push_back(ly);	verts2.push_back(uz);

			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(lz);
			verts2.push_back(ux);	verts2.push_back(ly);	verts2.push_back(uz);

			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(lz);
			verts2.push_back(lx);	verts2.push_back(uy);	verts2.push_back(uz);

			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(lz);
			verts2.push_back(ux);	verts2.push_back(uy);	verts2.push_back(uz);

			l_addNode(n->child1);
			l_addNode(n->child2);
		}
	};

	l_addNode(root);

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
	glLineWidth(2);
	glUniform3f(shaderPrograms_["wireframe"].getUniformLocation("wireframeColor"), 1, 0, 0);
	glDrawArrays(GL_LINES, 0, verts.size() / 3);

	// Disable buffers
	glDisableVertexAttribArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	// Write data
	glBindBuffer(GL_ARRAY_BUFFER, vBuffer);
	glBufferData(GL_ARRAY_BUFFER, verts2.size() * sizeof(float), verts2.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	// Enable buffers
	glEnableVertexAttribArray(0);

	// Render
	glLineWidth(2);
	glUniform3f(shaderPrograms_["wireframe"].getUniformLocation("wireframeColor"), 0, 1, 0);
	glDrawArrays(GL_LINES, 0, verts2.size() / 3);

	// Disable buffers
	glDisableVertexAttribArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBindVertexArray(0);
	glDisable(GL_DEPTH_TEST);

	// Delete VAO and buffer
	glDeleteBuffers(1, &vBuffer);
	glDeleteVertexArrays(1, &vao);


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

			/*
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
			//*

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
