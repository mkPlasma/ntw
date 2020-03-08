#pragma once

/*
 *	renderer.h
 *	
 *	Batches objects and renders them through OpenGL calls.
 *	
 */

#include"core/options.h"
#include"graphics/shaderProgram.h"
#include"core/world.h"
#include"math/matrix.h"
#include<unordered_map>

using std::unordered_map;


class Renderer{
	struct ObjectBatch{

		// Object data
		vector<Object*> objects;
		RenderType renderType;
		Material* material;

		// VAO data
		GLuint vaoID;
		int numVertices;
		vector<unsigned int> bufferIDs;
	};


	GraphicsOptions& gOptions_;

	ShaderProgram shaderBasic_;
	int timeUniformLoc_;
	int viewProjUniformLoc_;
	int viewPosUniformLoc_;
	int modelUniformLoc_;
	ShaderProgram shaderWireframe_;

	// Identity model matrix, do not change
	const Matrix identity_;


	// Current rendered world
	World* world_;

	// World object batches
	vector<ObjectBatch> objectBatches_;

public:
	Renderer(GraphicsOptions& gOptions);

	void init();
	void destroy();

	void initWorldRendering(World* world);
	void cleanupWorldRendering();

	void renderWorld(int time, float delta);
};
