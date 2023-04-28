#version 330

layout (location = 0) in vec4 vertex;
out vec2 texCoord;
uniform mat4 MVP;

void main()
{
    gl_Position = MVP * vec4(vertex.xy, 0.0, 1.0);
    texCoord = vertex.zw;
}
