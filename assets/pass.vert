#version 150

uniform mat4 ciModelViewProjection;
in vec4 ciPosition;
out vec3 color;

void main() {
	gl_Position = ciModelViewProjection * ciPosition;
	color = ciPosition.xyz;
}