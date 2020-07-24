#version 330 core

uniform sampler2D tex;
uniform int time;

in vec2 fTexCoords;

out vec4 fragColor;


void main(){
	fragColor = texture(tex, fTexCoords);
}