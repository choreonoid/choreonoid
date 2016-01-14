#version 330

layout (location = 0) in vec3 VertexPosition;
layout (location = 1) in vec3 VertexNormal;

out vec3 Position;
out vec3 Normal;

layout(shared) uniform MatrixBlock {
    mat4 modelViewMatrix;
    mat3 normalMatrix;
    mat4 MVP;
};

void main()
{
    Normal = normalize(normalMatrix * VertexNormal);
    Position = vec3(modelViewMatrix * vec4(VertexPosition, 1.0));

    gl_Position = MVP * vec4(VertexPosition, 1.0);
}
