#version 330

in vec3 normal; // interpolated normal

uniform int numLights;

struct LightInfo {
    vec3 direction;
    vec3 intensity;
    vec3 ambientIntensity;
};

uniform LightInfo lights[2];

uniform vec3 diffuseColor;
uniform float ambientIntensity;
uniform vec3 highlightColor = vec3(1.0, 1.0, 1.0);
uniform bool isHighlightEnabled = false;

layout(location = 0) out vec3 color;

void main()
{
    vec3 baseColor = diffuseColor;

    // Highlight mode: replace color with highlight color while preserving texture pattern
    if(isHighlightEnabled) {
        float maxComponent = max(max(baseColor.r, baseColor.g), baseColor.b);
        // Map brightness: 0 -> 0.5, 1 -> 1.0 (preserves texture pattern with boosted dark areas)
        baseColor = highlightColor * (0.5 + maxComponent * 0.5);
    }

    color = vec3(0.0, 0.0, 0.0);
    for(int i=0; i < numLights; ++i){
        LightInfo light = lights[i];
        vec3 n;
        if(gl_FrontFacing){
            n = normalize(normal);
        } else {
            n = -normalize(normal);
        }
        color += light.intensity * baseColor * max(dot(light.direction, n), 0.0);
        color += light.ambientIntensity * ambientIntensity * baseColor;
    }
}
