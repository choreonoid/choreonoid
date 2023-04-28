#version 330
//#version 400

// See the description written in the corresponding geometry shader code
// #define USE_DOUBLE_PRECISION_IN_WIREFRAME_RENDERING 1

#define MAX_NUM_LIGHTS 20
#define MAX_NUM_SHADOWS 2

#define USE_BLINN_PHONG_MODEL 1

in VertexData {
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

} inData;

/*
uniform MaterialBlock {
    vec3 diffuseColor;
    vec3 ambientColor;
    vec3 specularColor;
    float specularExponent;
};
*/

uniform vec3 diffuseColor;
uniform vec3 ambientColor;
uniform vec3 specularColor;
uniform vec3 emissionColor;
uniform float specularExponent;
uniform float alpha = 1.0;

uniform int numLights;

struct LightInfo {
    // the fourth element is 0.0 if the light is a directional light
    vec4 position;
    vec3 intensity;
    vec3 ambientIntensity;
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    // The following value is 0.0 if the light is not a spot light
    float cutoffAngle;
    float beamWidth;
    float cutoffExponent;
    vec3 direction;
};

uniform LightInfo lights[MAX_NUM_LIGHTS];

vec3 reflectionElements[MAX_NUM_LIGHTS];

uniform bool isTextureEnabled;
uniform sampler2D colorTexture;
uniform bool isVertexColorEnabled;
uniform vec3 fogColor;
uniform float maxFogDist;
uniform float minFogDist;
uniform bool isFogEnabled = false;

uniform bool isWireframeEnabled;
uniform vec4 wireframeColor;
uniform float wireframeWidth;

uniform int numShadows;

struct ShadowInfo {
    int lightIndex;
    sampler2DShadow shadowMap;
};

/*
  All the shadowMap variable values in the following array must be valid for some GPUs
  even if the valid number of shadow lights are less than the maximum number of shadows
  and not all the shdaowMaps are used in rendering. The proprietary drivers of Radeon GPUs
  require this condition. To achieve it, the initializeFrameRendering function of
  PhongShadowLightingProgram sets all the variable values. Note that for GPUs (drivers)
  including NVIDIA GPUs and Intel GPUs, you don't have to set a valid value to unused
  shadowMap variables, but the same implementation is used for those GPUs.
*/
uniform ShadowInfo shadows[MAX_NUM_SHADOWS];

uniform bool isShadowAntiAliasingEnabled;

layout(location = 0) out vec4 color4;

vec3 calcDiffuseAndSpecularElements(LightInfo light, vec3 diffuseColor);
void renderWireframe();

void main()
{
    vec3 color;
    float alpha2;

    if(isTextureEnabled){
        vec4 texColor4 = texture(colorTexture, inData.texCoord);
        vec3 texColor = vec3(texColor4);
        alpha2 = alpha * texColor4.a;
        color = emissionColor * texColor;
        for(int i=0; i < numLights; ++i){
            reflectionElements[i] = calcDiffuseAndSpecularElements(lights[i], texColor);
            color += lights[i].ambientIntensity * ambientColor * texColor;
        }
    } else {
        vec3 baseColor;
        if(isVertexColorEnabled){
            baseColor = inData.colorV;
        } else {
            baseColor = diffuseColor;
        }
        alpha2 = alpha;
        color = emissionColor;
        for(int i=0; i < numLights; ++i){
            reflectionElements[i] = calcDiffuseAndSpecularElements(lights[i], baseColor);
            color += lights[i].ambientIntensity * ambientColor;
        }
    }

    for(int i=0; i < numShadows; ++i){
        float shadow;
        vec4 shadowCoord = inData.shadowCoords[i];
        if(isShadowAntiAliasingEnabled){
            shadow  = textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2(-1, -1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2(-1,  1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2( 1,  1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2( 1, -1));
            shadow *= 0.25;
        } else {
            shadow = textureProj(shadows[i].shadowMap, shadowCoord);
        }
        reflectionElements[shadows[i].lightIndex] *= shadow;
    }

    for(int i=0; i < numLights; ++i){
        color += reflectionElements[i];
    }

    color4 = vec4(color, alpha2);

    if(isWireframeEnabled){
        renderWireframe();
    }

    if(isFogEnabled){
        float dist = abs(inData.position.z);
        float f = (maxFogDist - dist) / (maxFogDist - minFogDist);
        f = clamp(f, 0.0, 1.0);
        color4 = mix(vec4(fogColor, 1.0), color4, f);
    }

}


vec3 calcDiffuseAndSpecularElements(LightInfo light, vec3 diffuseColor)
{
    if(light.position.w == 0.0){
        // directional light
        vec3 s = normalize(vec3(light.position));
        vec3 v = normalize(vec3(-inData.position));
        vec3 n = normalize(inData.normal);
        if(!gl_FrontFacing){
            n = -n;
        }

#if USE_BLINN_PHONG_MODEL
        vec3 h = normalize(v + s);
#else
        vec3 h = reflect(-s, n); // Original phong model
#endif
        // Epsilon value is used in the max function because pow(0, 0) is not defined in GLSL
        vec3 spec = specularColor * pow(max(dot(h, n), 1.0e-6), specularExponent);
        
        return light.intensity * (diffuseColor * max(dot(s, n), 0.0) + spec);
        
    } else {
        // point light
        vec3 l = vec3(light.position) - inData.position;
        vec3 s = normalize(l);
        float ki;

        if(light.cutoffAngle == 0.0){ 
            ki = 1.0;
        } else {
            // spot light            
            vec3 direction = normalize(light.direction);
            float sd = dot(-s, direction);
            float angle = acos(sd);
            if(angle >= light.cutoffAngle) {
                return vec3(0.0);
            } else if(angle <= light.beamWidth){
                ki = 1.0;
            } else {
                float t = (light.cutoffAngle - angle) / (light.cutoffAngle - light.beamWidth);
                ki = pow(t, light.cutoffExponent);
            }
        }
        
        vec3 v = normalize(vec3(-inData.position));
        vec3 n = normalize(inData.normal);
        if(!gl_FrontFacing){
            n = -n;
        }
        float distance = sqrt(dot(l, l)); // l.length()
        ki *= 1.0 / max(1.0,
                        light.constantAttenuation +
                        distance * light.linearAttenuation +
                        distance * distance * light.quadraticAttenuation);
        
#if USE_BLINN_PHONG_MODEL
        vec3 h = normalize(v + s);
#else
        vec3 h = reflect(-s, n); // Original phong model
#endif
        // Epsilon value is used in the max function because pow(0, 0) is not defined in GLSL
        vec3 spec = specularColor * pow(max(dot(h, n), 1.0e-6), specularExponent);
        
        return ki * light.intensity * (diffuseColor * max(dot(s, n), 0.0) + spec);
    }
}


void renderWireframe()
{
    float edgeDistance;
    
    if(inData.edgeSituation == 0){
        edgeDistance = min(inData.edgeDistance.x, inData.edgeDistance.y);
        edgeDistance = min(edgeDistance, inData.edgeDistance.z);

    } else {

#ifndef USE_DOUBLE_PRECISION_IN_WIREFRAME_RENDERING
        
        vec2 fpos = vec2(gl_FragCoord);
        vec2 AF = fpos - inData.vertexA;
        float sqAF = dot(AF, AF);
        float AFcosA = dot(AF, inData.directionA);
        float d = abs(sqAF - AFcosA * AFcosA);

        vec2 BF = fpos - inData.vertexB;
        float sqBF = dot(BF, BF);
        float BFcosB = dot(BF, inData.directionB);
        d = min(d, abs(sqBF - BFcosB * BFcosB));
       
        // Only need to care about the 3rd edge for some cases.
        if(inData.edgeSituation == 1 || inData.edgeSituation == 2 || inData.edgeSituation == 4){
            float AFcosA0 = dot(AF, normalize(inData.vertexB - inData.vertexA));
            d = min(d, abs(sqAF - AFcosA0 * AFcosA0));
        }
        edgeDistance = sqrt(d);
#else
        dvec2 fpos = dvec2(gl_FragCoord);
        dvec2 AF = fpos - inData.vertexA;
        double sqAF = dot(AF, AF);
        double AFcosA = dot(AF, inData.directionA);
        double d = abs(sqAF - AFcosA * AFcosA);

        dvec2 BF = fpos - inData.vertexB;
        double sqBF = dot(BF, BF);
        double BFcosB = dot(BF, inData.directionB);
        d = min(d, abs(sqBF - BFcosB * BFcosB));
       
        // Only need to care about the 3rd edge for some cases.
        if(inData.edgeSituation == 1 || inData.edgeSituation == 2 || inData.edgeSituation == 4){
            double AFcosA0 = dot(AF, normalize(inData.vertexB - inData.vertexA));
            d = min(d, abs(sqAF - AFcosA0 * AFcosA0));
        }
        edgeDistance = float(sqrt(d));
#endif
    }

    float mixVal = smoothstep(wireframeWidth + 1, wireframeWidth - 1, edgeDistance) * wireframeColor.a;
    color4.rgb = mix(color4.rgb, wireframeColor.rgb, mixVal);
}
