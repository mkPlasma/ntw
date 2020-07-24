#version 330 core

layout(location = 0) in vec3 position;
//layout(location = 1) in vec3 normal;

//out vec3 fPos;
//out vec3 fNormal;

uniform mat4 viewProj;


// Coordinate system fix
// Inverts y axis and swaps y and z axes
const mat4 coordFix = mat4(
	1.0,	0.0,	0.0,	0.0,
	0.0,	0.0,	-1.0,	0.0,
	0.0,	1.0,	0.0,	0.0,
	0.0,	0.0,	0.0,	1.0
);


void main(){
	//fPos = position;
	//fNormal = mat3(transpose(inverse(model))) * normal;
	
	gl_Position = (viewProj * coordFix) * vec4(position, 1.0);
}