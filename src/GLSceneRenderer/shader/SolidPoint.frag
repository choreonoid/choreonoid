#version 330

//#define DO_DEPTH_TEST_IN_GEOMETRY_SHADER 1
//#define DO_DOUBLE_DEPTH_CHECK 1

#ifndef DO_DEPTH_TEST_IN_GEOMETRY_SHADER
flat in vec3 pointCenter;
flat in float offsetFragCoord_z;
uniform sampler2D depthTexture;
#endif

#ifdef DO_DOUBLE_DEPTH_CHECK
uniform vec2 viewportSize;
#endif

layout(location = 0) out vec4 fragColor;

uniform vec3 color;

void main()
{
#ifndef DO_DEPTH_TEST_IN_GEOMETRY_SHADER
    float MRD = 1.0 / ((1 << 24) - 1);
    vec2 texCoord = (pointCenter.xy + 1.0) / 2.0;
    float depth = texture(depthTexture, texCoord).r;
    if(depth < offsetFragCoord_z - MRD){

#ifndef DO_DOUBLE_DEPTH_CHECK
        discard;
#else
        vec2 texCoord2 = gl_FragCoord.xy / viewportSize;
        float depth2 = texture2D(depthTexture, texCoord2).r;
        if(depth2 < (offsetFragCoord_z - MRD)){
            discard;
        }
#endif
    }
#endif
    
    fragColor = vec4(color, 1.0);
}
