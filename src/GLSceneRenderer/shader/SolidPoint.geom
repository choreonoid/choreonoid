#version 330

// The following option enables to read the depth texture in the geometry shader
// and skip the fragment shader if the point is behind another object to increase the performance,
// but the obtained depth values seem incorrect in some GPU.
// If the following option is disabled, the depth texture is read in the fragment shader and it seems
// to work well.
// #define DO_DEPTH_TEST_IN_GEOMETRY_SHADER 1

layout( points ) in;
layout( points, max_vertices = 1 ) out;

in vec4 offsetPosition[];

uniform bool isReversedDepth;

#ifdef DO_DEPTH_TEST_IN_GEOMETRY_SHADER
uniform sampler2D depthTexture;
#else
flat out vec3 pointCenter;
flat out float offsetFragCoord_z;
#endif

void main()
{
    // Convert from NDC z to depth buffer value
    // For reversed depth with [0,1] range: NDC z is already in [0,1], just use it directly
    // For standard depth with [-1,1] range: transform from [-1,1] to [0,1]
    float z_ndc = offsetPosition[0].z / offsetPosition[0].w;
    if(isReversedDepth){
        // NDC range is [0, 1] with glClipControl GL_ZERO_TO_ONE
        offsetFragCoord_z = z_ndc;
    } else {
        // NDC range is [-1, 1], convert to [0, 1]
        offsetFragCoord_z = (z_ndc + 1.0) / 2.0;
    }

#ifdef DO_DEPTH_TEST_IN_GEOMETRY_SHADER
    float MRD = 1.0 / ((1 << 24) - 1);
    vec4 pos = gl_in[0].gl_Position;
    vec3 pos_c = pos.xy / pos.w;
    vec2 texCoord = (pos_c.xy + vec2(1.0, 1.0)) / 2.0;
    float depth = texture(depthTexture, texCoord).r;
    if(isReversedDepth){
        // Reversed depth: larger values are closer
        if(offsetFragCoord_z + MRD > depth){
            gl_Position = pos;
            gl_PointSize = gl_in[0].gl_PointSize;
            EmitVertex();
            EndPrimitive();
        }
    } else {
        // Standard depth: smaller values are closer
        if(offsetFragCoord_z - MRD < depth){
            gl_Position = pos;
            gl_PointSize = gl_in[0].gl_PointSize;
            EmitVertex();
            EndPrimitive();
        }
    }
#else
    gl_Position = gl_in[0].gl_Position;
    gl_PointSize = gl_in[0].gl_PointSize;
    pointCenter = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
    EmitVertex();
    EndPrimitive();
#endif
}
