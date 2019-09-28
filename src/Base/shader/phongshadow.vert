#version 330

#define MAX_NUM_SHADOWS 2

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec3 vertexNormal;
layout (location = 2) in vec2 vertexTexCoord;
layout (location = 3) in vec3 vertexColor;

out vec3 position;
out vec3 normal;
out vec2 texCoord;
out vec3 colorV;
out vec4 shadowCoords[MAX_NUM_SHADOWS];


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
    normal = normalize(normalMatrix * vertexNormal);
    position = vec3(modelViewMatrix * vertexPosition);

    texCoord = vertexTexCoord;
    colorV = vertexColor;
    
    for(int i=0; i < numShadows; ++i){
        shadowCoords[i] = shadowMatrices[i] * vertexPosition;
    }
    
    gl_Position = MVP * vertexPosition;
}
