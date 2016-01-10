#version 150


in vec3 color;
out vec4 fragColor;

void main() {
	fragColor.rgb = color;
	fragColor.a = 1.;
}
