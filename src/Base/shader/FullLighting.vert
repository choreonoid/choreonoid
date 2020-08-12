#version 330

#define MAX_NUM_SHADOWS 2

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec3 vertexNormal;
layout (location = 2) in vec2 vertexTexCoord;
layout (location = 3) in vec3 vertexColor;

out VertexData {
    vec3 position;
    vec3 normal;
    vec2 texCoord;
    vec3 colorV;
    vec4 shadowCoords[MAX_NUM_SHADOWS];
} outData;

/*
  Uniform blocks make rendering slow in the Ubuntu platforms with Intel GPUs.
  If the following definition is disabled and the definition of usual uniform
  variables are enabled instead of it, GLSLSceneRenderer uses the latter one
  to pass the matrix values. For the Intel GPUs, this results in better performace.
*/
/*
layout(std140) uniform TransformBlock {
    mat4 modelViewMatrix;
    mat4 MVP;
    mat3 normalMatrix;
    mat4 shadowMatrix;
};
*/

uniform mat4 modelViewMatrix;
uniform mat4 MVP;
uniform mat3 normalMatrix;
uniform int numShadows;
uniform mat4 shadowMatrices[MAX_NUM_SHADOWS];

void main()
{
    outData.normal = normalize(normalMatrix * vertexNormal);
    outData.position = vec3(modelViewMatrix * vertexPosition);

    outData.texCoord = vertexTexCoord;
    outData.colorV = vertexColor;
    
    for(int i=0; i < numShadows; ++i){
        outData.shadowCoords[i] = shadowMatrices[i] * vertexPosition;
    }
    
    gl_Position = MVP * vertexPosition;
}
