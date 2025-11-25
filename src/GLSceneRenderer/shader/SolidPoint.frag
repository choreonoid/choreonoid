#version 330

// Unified SolidPoint shader supporting both MSAA and non-MSAA modes
// Uses uniform bool useMsaa to switch between sampler types at runtime

//#define DO_DEPTH_TEST_IN_GEOMETRY_SHADER 1
//#define DO_DOUBLE_DEPTH_CHECK 1

#ifndef DO_DEPTH_TEST_IN_GEOMETRY_SHADER
flat in vec3 pointCenter;
flat in float offsetFragCoord_z;
uniform sampler2D depthTexture2D;
uniform sampler2DMS depthTextureMS;
uniform bool useMsaa;
uniform ivec2 depthTextureSize;
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

    float depth;
    if(useMsaa){
        ivec2 pixelCoord = ivec2(texCoord * vec2(depthTextureSize));
        depth = texelFetch(depthTextureMS, pixelCoord, 0).r;
    } else {
        depth = texture(depthTexture2D, texCoord).r;
    }

    if(depth < offsetFragCoord_z - MRD){

#ifndef DO_DOUBLE_DEPTH_CHECK
        discard;
#else
        float depth2;
        if(useMsaa){
            ivec2 pixelCoord2 = ivec2(gl_FragCoord.xy);
            depth2 = texelFetch(depthTextureMS, pixelCoord2, 0).r;
        } else {
            vec2 texCoord2 = gl_FragCoord.xy / viewportSize;
            depth2 = texture(depthTexture2D, texCoord2).r;
        }
        if(depth2 < (offsetFragCoord_z - MRD)){
            discard;
        }
#endif
    }
#endif

    fragColor = vec4(color, 1.0);
}
