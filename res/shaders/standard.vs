#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texCoords;

out vec3 fPos;
out vec3 fNormal;
out vec2 fTexCoords;

uniform mat4 viewProj;
uniform mat4 model;


void main(){
	fPos = vec3(model * vec4(position, 1.0));
	fNormal = mat3(transpose(inverse(model))) * normal;
	fTexCoords = texCoords;
	
	
	gl_Position = viewProj * model * vec4(position, 1.0);
}