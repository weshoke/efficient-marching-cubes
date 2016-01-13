#version 150

uniform float uBrightness;

noperspective in vec3 pixDistance;
in vec4 vColor;
out vec4 fragColor;

void main() {
	// determine frag distance to closest edge
	float fNearest = min(min(pixDistance[0], pixDistance[1]), pixDistance[2]);
	float fEdgeIntensity = exp2( -1.0 * fNearest * fNearest );

	// blend between edge color and face color
	vec3 vFaceColor = vColor.rgb;
	//vec3 vEdgeColor = vec3(.45, .8, .8);
	vec3 vEdgeColor = vec3(0.2);

	fragColor.rgb = mix( vFaceColor, vEdgeColor, fEdgeIntensity ) * uBrightness;
	fragColor.a = 1.;
}
