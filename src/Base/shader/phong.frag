#version 330

in vec3 Position;
in vec3 Normal;

//uniform MaterialBlock {
    uniform vec3 DiffuseColor;
    uniform vec3 AmbientColor;
    uniform vec3 SpecularColor;
    uniform float Shininess;
//};

uniform vec4 LightPosition;
uniform vec3 LightIntensity;

layout(location = 0) out vec4 FragColor;

vec3 ads( )
{
    vec3 s;
    if(LightPosition.w == 0.0){
        s = normalize(vec3(LightPosition));
    } else {
        s = normalize(vec3(LightPosition) - Position);
    }
    
    vec3 v = normalize(vec3(-Position));
    vec3 r = reflect(-s, Normal);

    return
        LightIntensity *
        (AmbientColor +
         DiffuseColor * max(dot(s, Normal), 0.0) +
         SpecularColor * pow(max(dot(r, v), 0.0), Shininess));
}

void main() {
    FragColor = vec4(ads(), 1.0);
}
