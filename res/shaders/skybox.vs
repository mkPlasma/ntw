#version 330 core

layout(location = 0) in vec3 position;

out vec3 fTexCoords;

uniform mat4 viewProj;


void main(){
	fTexCoords = position;
	
	gl_Position = viewProj * vec4(position, 1.0);
	
	// Set depth to maximum
	gl_Position.z = gl_Position.w;
}