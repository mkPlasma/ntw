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
		vector<GLuint> bufferIds;
	};

	struct BatchGroup{
		string shaderProgram;
		vector<ObjectBatch> batches;
	};

	struct PortalBatch{
		Portal* portal;

		// VAO data
		GLuint vaoId;
		GLuint vBufferId;
	};

	struct Framebuffer{
		GLuint id;
		GLuint textureId;
		GLuint depthBufferId;
	};


	GraphicsOptions& gOptions_;

	unordered_map<string, ShaderProgram> shaderPrograms_;

	// Screen rendering
	GLuint screenVao_;
	GLuint screenVBuffer_;

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

	// Skybox VAO and buffer indices
	GLuint skyboxVao_;
	GLuint skyboxVBuffer_;


	Framebuffer createFrameBuffer(int width, int height, bool useMipMap = false, bool useDepthBuffer = false, bool useStencilBuffer = false, int msaaSamples = -1);


	// World rendering
	void addObjectToBatchGroups(Object* object);

	ObjectBatch& addObjectBatch(BatchGroup& group, Object* object);
	void initObjectBatch(ObjectBatch& batch);

	void addPortalBatch(Portal* portal);

	void setViewProj(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly);
	void setViewProj(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly, Vec3 clipPlaneNormal, float clipPlaneDistance);
	void setViewProjSub(const Camera& camera, Matrix& viewProj, Matrix& viewProjRotOnly);

	// Render world, then portals, recursively
	void renderWorld(int time, float physTimeDelta, int iterations);

	// Render all world elements except for portals
	void renderWorldSub(const Camera& camera, const Matrix& viewProj, const Matrix& viewProjRotOnly, int time, float physTimeDelta);

public:
	Renderer(GraphicsOptions& gOptions);

	void init();
	void destroy();

	void render(int time);


	// World rendering
	void addObject(Object* object);

	void initWorldRendering(World* world);
	void cleanupWorldRendering();

	// Complete world render function
	void renderWorld(int time, float physTimeDelta);
};
