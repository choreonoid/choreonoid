#version 330

layout (location = 0) in vec3 vertexPosition;
layout (location = 3) in vec3 vertexColor;

out VertexData {
    vec3 color;
} outData;

uniform mat4 MVP;
uniform float pointSize = 1.0;

void main()
{
    gl_Position = MVP * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;
    outData.color = vertexColor;
}
