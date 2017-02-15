#version 330

in float transparency;

layout (location = 0) out vec4 fragColor;

void main()
{
    fragColor = vec4(1.0, 1.0, 1.0, 1.0);
    fragColor.a *= transparency;
}
