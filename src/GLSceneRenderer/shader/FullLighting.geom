/**
   This geometry shader implements the solid wireframe method described in the following paper:
   Solid Wireframe, NVIDIA Whitepaper WP-03014-001_v01.
*/   

#version 330
//#version 400

// Enable the following macro to improve the wireframe rendering. When a vertex of an edge is behind
// the viewpoint, a special algorithm is used to render the edge, but the edge is sometimes not
// clearly rendered due to the lack of the presition in calculating the distance between the
// flagment and the edge. This can be improved by using the double precision.
// Note that the same opition must be enabled in the fragment shader and the option requries the
// GLSL version 4.0 or later.
// #define USE_DOUBLE_PRECISION_IN_WIREFRAME_RENDERING 1

#define MAX_NUM_SHADOWS 2

layout( triangles ) in;
layout( triangle_strip, max_vertices = 3 ) out;

out VertexData {
    vec3 position;
    vec3 normal;
    vec2 texCoord;
    vec3 colorV;
    vec4 shadowCoords[MAX_NUM_SHADOWS];

    flat int edgeSituation;
    noperspective vec3 edgeDistance;

#ifndef USE_DOUBLE_PRECISION_IN_WIREFRAME_RENDERING
    flat vec2 vertexA;
    flat vec2 directionA;
    flat vec2 vertexB;
    flat vec2 directionB;
#else
    flat dvec2 vertexA;
    flat dvec2 directionA;
    flat dvec2 vertexB;
    flat dvec2 directionB;
#endif

} outData;

in VertexData {
    vec3 position;
    vec3 normal;
    vec2 texCoord;
    vec3 colorV;
    in vec4 shadowCoords[MAX_NUM_SHADOWS];
} inData[];

uniform bool isWireframeEnabled;
uniform mat4 viewportMatrix;

int infoA[]  = int[]( 0, 0, 0, 0, 1, 1, 2 );
int infoB[]  = int[]( 1, 1, 2, 0, 2, 1, 2 );
int infoAd[] = int[]( 2, 2, 1, 1, 0, 0, 0 );
int infoBd[] = int[]( 2, 2, 1, 2, 0, 2, 1 );

void outputVertex(int i);
void makeEdgeDataForWireframeRendering();

void main()
{
    if(isWireframeEnabled){
        makeEdgeDataForWireframeRendering();
    } else {
        for(int i=0; i < 3; ++i){
            outputVertex(i);
        }
    }

    EndPrimitive();
}


void outputVertex(int i)
{
    outData.position = inData[i].position;
    outData.normal = inData[i].normal;
    outData.texCoord = inData[i].texCoord;
    outData.colorV = inData[i].colorV;
    outData.shadowCoords = inData[i].shadowCoords;
    gl_Position = gl_in[i].gl_Position;
    EmitVertex();
}


void makeEdgeDataForWireframeRendering()
{
    int situation = 0;
    
    vec2 p[3];
    for(int i=0; i < 3; ++i){
        vec4 pos = gl_in[i].gl_Position;
        if(pos.z >= -pos.w){
            p[i] = vec2(viewportMatrix * (pos / pos.w));
        } else {
            situation += (4 >> i);
        }
    }

    if(situation == 7){
        return;
    }
    outData.edgeSituation = situation;

    float h[3];
    if(situation == 0){
        float a = length(p[1] - p[2]);
        float b = length(p[2] - p[0]);
        float c = length(p[1] - p[0]);
        float alpha = acos((b*b + c*c - a*a) / (2.0*b*c));
        float beta = acos((a*a + c*c - b*b) / (2.0*a*c));
        h[0] = abs(c * sin(beta));
        h[1] = abs(c * sin(alpha));
        h[2] = abs(b * sin(alpha));

        for(int i=0; i < 3; ++i){
            outData.edgeDistance = vec3(0);
            outData.edgeDistance[i] = h[i];
            outputVertex(i);
        }

    } else {
        // Processing the case where some vertices are behind the view volume
        // A, B : Position of a valid vertex in the clip coordinate system space
        //  Note that gl_Position is a value in the clip coordinate system.
        // C, D : Position of a vertex behind the view point in the clip space
        // C_clipped, D_clipped : Position of C, D clipped by a near plane in the cilp space
        //  For the vertex A and C, find the interpolation point where elements C.z equals C.w
        //  so that the point can satisfy the condition that -C.w <= C.z <= C.w.
        //  The interpolation is applied for all the emements x, y, z, w of the point vector in
        //  the clip space.
        // gDirectionA, gDirectionB : Direction from a valid vertex to another vertex

#ifndef USE_DOUBLE_PRECISION_IN_WIREFRAME_RENDERING
        
        outData.vertexA = p[infoA[situation]];
        vec4 A = gl_in[infoA[situation]].gl_Position;
        vec4 C = gl_in[infoAd[situation]].gl_Position;
        float r = 1.0 / (1.0 - ((C.z + C.w) / (A.z + A.w)));
        vec4 C_clipped = r * C + (1.0 - r) * A;
        vec2 C_clipped_viewport = vec2(viewportMatrix * C_clipped / C_clipped.w);
        outData.directionA = normalize(C_clipped_viewport - outData.vertexA);
        
        outData.vertexB = p[infoB[situation]];
        vec4 B = gl_in[infoB[situation]].gl_Position;
        vec4 D = gl_in[infoBd[situation]].gl_Position;
        r = 1.0 / (1.0 - ((D.z + D.w) / (B.z + B.w)));
        vec4 D_clipped = r * D + (1.0 - r) * B;
        vec2 D_clipped_viewport = vec2(viewportMatrix * D_clipped / D_clipped.w);
        outData.directionB = normalize(D_clipped_viewport - outData.vertexB);
#else
        outData.vertexA = dvec2(p[infoA[situation]]);
        dvec4 A = dvec4(gl_in[infoA[situation]].gl_Position);
        dvec4 C = dvec4(gl_in[infoAd[situation]].gl_Position);
        double r = 1.0 / (1.0 - ((C.z + C.w) / (A.z + A.w)));
        dvec4 C_clipped = r * C + (1.0 - r) * A;
        dvec2 C_clipped_viewport = dvec2(viewportMatrix * vec4(C_clipped / C_clipped.w));
        outData.directionA = normalize(C_clipped_viewport - outData.vertexA); 
        
        outData.vertexB = p[infoB[situation]];
        dvec4 B = dvec4(gl_in[infoB[situation]].gl_Position);
        dvec4 D = dvec4(gl_in[infoBd[situation]].gl_Position);
        r = 1.0 / (1.0 - ((D.z + D.w) / (B.z + B.w)));
        dvec4 D_clipped = r * D + (1.0 - r) * B;
        dvec2 D_clipped_viewport = dvec2(viewportMatrix * vec4(D_clipped / D_clipped.w));
        outData.directionB = normalize(D_clipped_viewport - outData.vertexB);
#endif        

        for(int i=0; i < 3; ++i){
            outputVertex(i);
        }
    }
}
