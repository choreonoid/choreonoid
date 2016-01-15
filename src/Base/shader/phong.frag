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

uniform vec4 lightPosition;
uniform vec3 lightIntensity;

layout(location = 0) out vec4 color;

vec3 ads( )
{
    vec3 s;
    if(lightPosition.w == 0.0){
        s = normalize(vec3(lightPosition));
    } else {
        s = normalize(vec3(lightPosition) - position);
    }
    
    vec3 v = normalize(vec3(-position));
    vec3 r = reflect(-s, normal);

    return
        lightIntensity *
        (ambientColor +
         diffuseColor * max(dot(s, normal), 0.0) +
         specularColor * pow(max(dot(r, v), 0.0), shininess))
        + emissionColor;
}

void main() {
    color = vec4(ads(), 1.0);
}
