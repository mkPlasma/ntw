#include"renderer.h"

#include"objects/modelFunc.h"
#include"math/mathFunc.h"
#include"physics/physDefine.h"
#include<math.h>

using ntw::toRadians;


Renderer::Renderer(GraphicsOptions& gOptions) : gOptions_(gOptions) {

}

void Renderer::init(){

	// Load shader programs
	screenShader_.compile("screen.vs", "screen.fs");
	screenShader_.link();

	ShaderProgram shaderStandard = ShaderProgram("standard");
	shaderStandard.compile("standard.vs", "standard.fs");
	shaderStandard.link();
	shaderPrograms_.emplace(shaderStandard.getName(), shaderStandard);

	ShaderProgram shaderPortal = ShaderProgram("portal");
	shaderPortal.compile("portal.vs", "portal.fs");
	shaderPortal.link();
	shaderPrograms_.emplace(shaderPortal.getName(), shaderPortal);

	ShaderProgram shaderWireframe = ShaderProgram("wireframe");
	shaderWireframe.compile("wireframe.vs", "wireframe.fs");
	shaderWireframe.link();
	shaderPrograms_.emplace(shaderWireframe.getName(), shaderWireframe);


	// Screen quad
	const float vertices[] = {
		0, 0,
		0, 1,
		1, 0,

		0, 1,
		1, 1,
		1, 0
	};

	glGenVertexArrays(1, &screenVAO);
	glBindVertexArray(screenVAO);

	// Create vertex buffer
	glGenBuffers(1, &screenVBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, screenVBuffer);

	// Write data
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);


	// Create screen framebuffer
	fbScreen_ = createFrameBuffer(gOptions_.resolutionX, gOptions_.resolutionY);
}

void Renderer::destroy(){

	// Destroy shader programs
	for(auto i = shaderPrograms_.begin(); i != shaderPrograms_.end(); i++)
		i->second.destroy();

	shaderPrograms_.clear();

	// Delete screen quad buffers
	glDeleteBuffers(1, &screenVBuffer);
	glDeleteVertexArrays(1, &screenVAO);

	// Delete screen framebuffer
	glDeleteTextures(1, &fbScreen_.textureId);
	glDeleteRenderbuffers(1, &fbScreen_.depthBufferId);
	glDeleteFramebuffers(1, &fbScreen_.id);
}

Renderer::Framebuffer Renderer::createFrameBuffer(int width, int height, bool useMipMap, bool useDepthBuffer, int msaaSamples){

	// Create and bind framebuffer
	GLuint bufferId;
	glGenFramebuffers(1, &bufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, bufferId);

	// Create texture
	GLuint textureId;
	int texType = msaaSamples == -1 ? GL_TEXTURE_2D : GL_TEXTURE_2D_MULTISAMPLE;

	glGenTextures(1, &textureId);
	glBindTexture(texType, textureId);

	if(msaaSamples == -1){
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(texType, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(texType, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}
	else
		glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, msaaSamples, GL_RGB, width, height, GL_TRUE);

	//glTexParameteri(texType, GL_GENERATE_MIPMAP, useMipMap ? GL_TRUE : GL_FALSE);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texType, textureId, 0);
	glBindTexture(texType, 0);


	// Create depth buffer
	GLuint depthBufferId = -1;

	if(useDepthBuffer){
		glGenRenderbuffers(1, &depthBufferId);
		glBindRenderbuffer(GL_RENDERBUFFER, depthBufferId);

		if(msaaSamples == -1)
			glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
		else
			glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaaSamples, GL_DEPTH_COMPONENT, width, height);

		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBufferId);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);
	}

	// Error 
	auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		ntw::error("Framebuffer failed to generate completely!");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return {bufferId, textureId, depthBufferId};
}

void Renderer::render(int time){

	screenShader_.use();
	glUniform1i(screenShader_.getUniformLocation("time"), time);

	// Blit world framebuffer to screen framebuffer
	glEnable(GL_MULTISAMPLE);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, fbWorld_.id);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbScreen_.id);
	glBlitFramebuffer(0, 0, gOptions_.resolutionX, gOptions_.resolutionY, 0, 0, gOptions_.resolutionX, gOptions_.resolutionY, GL_COLOR_BUFFER_BIT, GL_LINEAR);

	glDisable(GL_MULTISAMPLE);


	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT);

	// Set screen quad texture
	glBindTexture(GL_TEXTURE_2D, fbScreen_.textureId);

	glBindVertexArray(screenVAO);

	// Render
	glEnableVertexAttribArray(0);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glDisableVertexAttribArray(0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);
}
