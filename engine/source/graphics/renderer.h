#pragma once

/*
 *	renderer.h
 *	
 *	Batches objects and renders them through OpenGL calls.
 *	
 */

#include"core/options.h"
#include"shaderProgram.h"
#include"core/world.h"
#include"math/matrix.h"
#include<unordered_map>

using std::unordered_map;


struct ObjectBatch{

	// Object data
	int renderType;
	vector<Object*> objects;

	// VAO data
	int vaoID;
	int textureID;
	int numVertices;
	vector<unsigned int> bufferIDs;
};


class Renderer{

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

	// Map for textures and their IDs
	unordered_map<Texture*, int> worldTextures_;

	// World object batches
	vector<ObjectBatch> objectBatches_;

public:
	Renderer(GraphicsOptions& gOptions);

	void init();
	void destroy();

	void initWorldRendering(World* world);
	void cleanupWorldRendering();

	void renderWorld(const int& time, const float& delta);
};
