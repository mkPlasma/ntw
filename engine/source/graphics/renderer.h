#pragma once

/*
 *	renderer.h
 *	
 *	Batches objects and renders them through OpenGL calls.
 *	
 */

class Renderer;

#include"core/options.h"
#include"graphics/shaderProgram.h"
#include"graphics/renderType.h"
#include"graphics/camera.h"
#include"objects/material.h"
#include"objects/portal.h"
#include"core/world.h"
#include"math/matrix.h"
#include<unordered_map>

using std::unordered_map;

class Object;

#define NTW_NEAR_CLIP	0.01f
#define NTW_FAR_CLIP	1000


class Renderer{
	struct ObjectBatch{

		// Object data
		vector<Object*> objects;
		RenderType renderType;
		Material* material;

		// VAO data
		GLuint vaoId;
		int numVertices;
		vector<unsigned int> bufferIds;
	};

	struct BatchGroup{
		string shaderProgram;
		vector<ObjectBatch> batches;
	};

	struct PortalBatch{
		Portal* portal;

		// VAO data
		GLuint vaoId;
		vector<unsigned int> bufferIds;
	};

	struct Framebuffer{
		GLuint id;
		GLuint textureId;
		GLuint depthBufferId;
	};


	GraphicsOptions& gOptions_;

	unordered_map<string, ShaderProgram> shaderPrograms_;

	// Screen rendering
	GLuint screenVAO;
	GLuint screenVBuffer;

	ShaderProgram screenShader_;

	Framebuffer fbScreen_;


	// Identity model matrix, do not change
	const Matrix identity_;


	// Current rendered world
	World* world_;

	// World framebuffers
	Framebuffer fbWorld_;
	Framebuffer fbWorldPortal_;

	// World object batches
	vector<BatchGroup> objectBatchGroups_;

	// World portal batches
	vector<PortalBatch> portalBatches_;


	// World rendering
	void addObjectToBatchGroups(Object* object);

	ObjectBatch& addObjectBatch(BatchGroup& group, Object* object);
	void initObjectBatch(ObjectBatch& batch);

	void addPortalBatch(Portal* portal);

	Matrix getViewProj(const Camera& camera);
	Matrix getViewProj(const Camera& camera, Vec3 clipPlaneNormal, float clipPlaneDistance);
	Matrix getViewProjSub(const Camera& camera);


public:
	Renderer(GraphicsOptions& gOptions);

	void init();
	void destroy();

	Framebuffer createFrameBuffer(int width, int height, bool useMipMap = false, bool useDepthBuffer = false, int msaaSamples = -1);

	void render(int time);


	// World rendering
	void addObject(Object* object);

	void initWorldRendering(World* world);
	void cleanupWorldRendering();

	// Complete world render function
	void renderWorld(int time, float physTimeDelta);

	// Render all world elements except for portals
	void renderWorldSub(const Camera& camera, int time, float physTimeDelta);
	void renderWorldSub(const Camera& camera, const Matrix& viewProj, int time, float physTimeDelta);
};
