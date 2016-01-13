#version 150

uniform mat4 ciModelViewProjection;
in vec4 ciPosition;
in vec4 ciColor;

out vec4 color;

void main() {
	color = ciColor;
	gl_Position = ciModelViewProjection * ciPosition;
}
