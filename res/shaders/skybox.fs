#version 330 core

//uniform sampler2D tex;
uniform int time;

in vec3 fTexCoords;

out vec4 fragColor;


void main(){
	fragColor = vec4(0.0, 0.0, 0.0, 1.0);
}