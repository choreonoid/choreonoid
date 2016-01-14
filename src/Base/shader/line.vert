#version 330

layout (location = 0) in vec3 VertexPosition;

uniform MatrixBlock {
    mat4 ModelViewMatrix;
    mat3 NormalMatrix;
    mat4 MVP;
}

void main()
{
    gl_Position = MVP * vec4(VertexPosition, 1.0);
}
