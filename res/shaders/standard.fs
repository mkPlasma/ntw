#version 330 core

uniform sampler2D tex;
uniform int time;
uniform vec3 viewPos;

in vec3 fPos;
in vec3 fNormal;
in vec2 fTexCoords;

out vec4 fragColor;


const vec3 lightPos = vec3(0.0, 0.0, 5.0);
const vec3 lightColor = vec3(0.9, 0.9, 0.7);
const float lightPower = 30.0;
const float specularAmt = 0.5;

const float gamma = 2.2;

void main(){
	
	vec3 normal = normalize(fNormal);
	
	vec3 lightDir	= normalize(lightPos - fPos);
	vec3 viewDir	= normalize(viewPos - fPos);
	vec3 halfwayDir	= normalize(lightDir + viewDir);
	
	
	float lightDist = distance(fPos, lightPos);
	float lightAttenuation = 1.0 / (pow(lightDist, 2.0) + 0.5);
	
	float diffuse = max(dot(normal, lightDir), 0.0);
	float specular = pow(max(dot(normal, halfwayDir), 0.0), 64.0) * specularAmt;
	
	vec3 light = (diffuse + specular) * lightAttenuation * lightPower * lightColor;
	light += vec3(0.1, 0.1, 0.1);
	
	// Object color
	fragColor = texture(tex, fTexCoords);
	
	// Apply lighting
	fragColor.rgb *= light;
	
	// Gamma correction
	fragColor.rgb = pow(fragColor.rgb, vec3(1.0 / gamma));
}