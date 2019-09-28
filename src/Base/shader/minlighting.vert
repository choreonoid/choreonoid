#version 330

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec3 vertexNormal;

out vec3 normal;

uniform mat4 MVP;
uniform mat3 normalMatrix;

void main()
{
    normal = normalMatrix * vertexNormal;
    gl_Position = MVP * vertexPosition;
}
