#version 330

layout (location = 0) in vec3 vertexPosition;

out vec4 offsetPosition;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform float pointSize = 5.0;
uniform float depthOffset = 0.001;

void main()
{
    vec4 p = modelViewMatrix * vec4(vertexPosition, 1.0);
    offsetPosition = projectionMatrix * (p + vec4(0, 0, depthOffset, 0.0));
    gl_Position = projectionMatrix * p;
    gl_PointSize = pointSize;
}
