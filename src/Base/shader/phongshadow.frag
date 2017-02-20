#version 330

#define MAX_NUM_SHADOWS 3

in vec3 position;
in vec3 normal;
in vec4 shadowCoords[MAX_NUM_SHADOWS];

/*
uniform MaterialBlock {
    vec3 diffuseColor;
    vec3 ambientColor;
    vec3 specularColor;
    float shininess;
};
*/

uniform vec3 diffuseColor;
uniform vec3 ambientColor;
uniform vec3 specularColor;
uniform vec3 emissionColor;
uniform float shininess;
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

uniform LightInfo lights[10];

vec3 reflectionElements[10];

uniform int numShadows;

struct ShadowInfo {
    int lightIndex;
    sampler2DShadow shadowMap;
};
uniform ShadowInfo shadows[3];
uniform bool isShadowAntiAliasingEnabled;

uniform vec3 fogColor;
uniform float maxFogDist;
uniform float minFogDist;
uniform bool isFogEnabled = false;

layout(location = 0) out vec4 color;

vec3 calcDiffuseAndSpecularElements(LightInfo light)
{
    if(light.position.w == 0.0){
        // directional light
        vec3 s = normalize(vec3(light.position));
        vec3 v = normalize(vec3(-position));
        vec3 n = normalize(normal);
        vec3 r = reflect(-s, n);
        return light.intensity * (
            diffuseColor * max(dot(s, n), 0.0) +
            specularColor * pow(max(dot(r, v), 0.0), shininess));
    } else {
        // point light
        vec3 l = vec3(light.position) - position;
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
        
        vec3 v = normalize(vec3(-position));
        vec3 n = normalize(normal);
        vec3 r = reflect(-s, n);
        //float distance = l.length();
        float distance = sqrt(dot(l, l));
        ki *= 1.0 / max(1.0,
                        light.constantAttenuation +
                        distance * light.linearAttenuation +
                        distance * distance * light.quadraticAttenuation);
        
        return ki * light.intensity * (
            diffuseColor * max(dot(s, n), 0.0) +
            specularColor * pow(max(dot(r, v), 0.0), shininess));
    }
}

void main()
{
    for(int i=0; i < numLights; ++i){
        reflectionElements[i] = calcDiffuseAndSpecularElements(lights[i]);
    }
    for(int i=0; i < numShadows; ++i){
        float shadow;
        if(isShadowAntiAliasingEnabled){
            vec4 shadowCoord = shadowCoords[i];
            shadow  = textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2(-1, -1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2(-1,  1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2( 1,  1));
            shadow += textureProjOffset(shadows[i].shadowMap, shadowCoord, ivec2( 1, -1));
            shadow *= 0.25;
        } else {
            shadow = textureProj(shadows[i].shadowMap, shadowCoords[i]);
        }
        reflectionElements[shadows[i].lightIndex] *= shadow;
    }
    vec3 c = emissionColor;
    for(int i=0; i < numLights; ++i){
        c += reflectionElements[i] + lights[i].ambientIntensity * ambientColor;
    }

    if(isFogEnabled){
        float dist = abs(position.z);
        float f = (maxFogDist - dist) / (maxFogDist - minFogDist);
        f = clamp(f, 0.0, 1.0);
        c = mix(fogColor, c, f);
    }
    
    color = vec4(c, alpha);
}
