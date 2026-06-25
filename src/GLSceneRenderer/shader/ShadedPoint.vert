#version 330

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec3 vertexNormal;
layout (location = 3) in vec3 vertexColor;

out VertexData {
    vec3 position;
    vec3 normal;
    vec3 colorV;
} outData;

uniform mat4 modelViewMatrix;
uniform mat4 MVP;
uniform mat3 normalMatrix;
uniform float pointSize = 1.0;

void main()
{
    outData.position = vec3(modelViewMatrix * vertexPosition);
    outData.normal = normalMatrix * vertexNormal;
    outData.colorV = vertexColor;

    gl_Position = MVP * vertexPosition;
    gl_PointSize = pointSize;
}
