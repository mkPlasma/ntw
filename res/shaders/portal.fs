#version 330 core

uniform sampler2DMS tex;
uniform int time;

//in vec3 fPos;
//in vec3 fNormal;

out vec4 fragColor;


vec4 textureSample(sampler2DMS texture, ivec2 coord){
	vec4 color = vec4(0.0);
	
	for(int i = 0; i < 4; i++)
		color += texelFetch(texture, coord, i);
	
	return color / float(4);
}

void main(){
	fragColor = textureSample(tex, ivec2(gl_FragCoord.x, gl_FragCoord.y));
	//fragColor.rb *= 1.5;
}