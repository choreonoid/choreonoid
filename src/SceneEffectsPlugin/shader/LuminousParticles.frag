#version 330

in vec3 position;
in float alpha;

layout (location = 0) out vec4 fragColor;

uniform sampler2D particleTex;

uniform vec3 fogColor;
uniform float maxFogDist;
uniform float minFogDist;
uniform bool isFogEnabled = false;

void main()
{
    vec4 texColor = texture(particleTex, gl_PointCoord);

    vec3 color = texColor.xyz;

    if(isFogEnabled){
        float dist = abs(position.z);
        float f = (maxFogDist - dist) / (maxFogDist - minFogDist);
        f = clamp(f, 0.0, 1.0);
        color = mix(fogColor, color, f);
    }

    fragColor = vec4(color, texColor.a * alpha);
}
