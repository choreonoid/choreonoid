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
uniform vec3 ambientColor;

layout(location = 0) out vec3 color;

void main()
{
    color = vec3(0.0, 0.0, 0.0);
    for(int i=0; i < numLights; ++i){
        LightInfo light = lights[i];
        vec3 n;
        if(gl_FrontFacing){
            n = normalize(normal);
        } else {
            n = -normalize(normal);
        }
        color += light.intensity * diffuseColor * max(dot(light.direction, n), 0.0);
        color += light.ambientIntensity * ambientColor;
    }
}
