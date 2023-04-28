#version 330

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec3 vertexNormal;

uniform mat4 MVP;
uniform mat3 normalMatrix;

void main()
{
    vec3 normal = normalize(normalMatrix * vertexNormal);
    gl_Position = MVP * vertexPosition + vec4(normal * 0.01, 0.0);
}
