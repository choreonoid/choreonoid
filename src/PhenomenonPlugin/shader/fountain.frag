#version 330

in float transparency;
uniform sampler2D particleTex;

layout (location = 0) out vec4 fragColor;

void main()
{
    fragColor = texture(particleTex, gl_PointCoord);
    fragColor.a *= transparency;
}
