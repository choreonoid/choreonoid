#version 330

layout (location = 0) in vec3 vertexPosition;

/*
uniform MatrixBlock {
    mat4 ModelViewMatrix;
    mat3 NormalMatrix;
    mat4 ModelViewProjectionMatrix;
};
*/

uniform mat4 MVP;

void main()
{
    gl_Position = MVP * vec4(vertexPosition, 1.0);
}
