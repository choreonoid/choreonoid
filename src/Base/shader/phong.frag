#version 330

in vec3 position;
in vec3 normal;

/*
uniform MaterialBlock {
    vec3 diffuseColor;
    vec3 ambientColor;
    vec3 specularColor;
    float shininess;
};
*/

uniform vec3 diffuseColor;
uniform vec3 ambientColor;
uniform vec3 specularColor;
uniform vec3 emissionColor;
uniform float shininess;

uniform int numLights;

struct LightInfo {
    vec4 position;
    vec3 intensity;
};
uniform LightInfo lights[10];

layout(location = 0) out vec4 color;

vec3 ads(LightInfo light)
{
    vec3 s;
    if(light.position.w == 0.0){
        s = normalize(vec3(light.position));
    } else {
        s = normalize(vec3(light.position) - position);
    }
    
    vec3 v = normalize(vec3(-position));
    vec3 r = reflect(-s, normal);

    return
        light.intensity *
        (ambientColor +
         diffuseColor * max(dot(s, normal), 0.0) +
         specularColor * pow(max(dot(r, v), 0.0), shininess))
        + emissionColor;
}

void main() {
    vec3 c = vec3(0.0);
    for(int i=0; i < numLights; ++i){
        c += ads(lights[i]);
    }
    color = vec4(c, 1.0);
}
