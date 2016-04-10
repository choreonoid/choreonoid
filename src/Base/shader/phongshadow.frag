#version 330

in vec3 position;
in vec3 normal;
in vec4 shadowCoord;

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
    float falloffAngle; 
    float falloffExponent;
    float beamWidth;
    vec3 direction;
};

uniform LightInfo lights[10];

uniform bool isShadowEnabled;
uniform bool isShadowAntiAliasingEnabled;
uniform int shadowLightIndex;
uniform sampler2DShadow shadowMap;

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

        if(light.falloffAngle == 0.0){ 
            ki = 1.0;
        } else {
            // spot light            
            vec3 direction = normalize(light.direction);
            float sd = dot(-s, direction);
            float angle = acos(sd);
            if(angle > light.falloffAngle) {
                return vec3(0.0);
            }
            ki = pow(sd, light.falloffExponent);
        }
        
        vec3 v = normalize(vec3(-position));
        vec3 n = normalize(normal);
        vec3 r = reflect(-s, normal);
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
    vec3 c = emissionColor;
    if(!isShadowEnabled){
        for(int i=0; i < numLights; ++i){
            c += calcDiffuseAndSpecularElements(lights[i]);
            c += lights[i].ambientIntensity * ambientColor;
        }
    } else {
        for(int i=0; i < numLights; ++i){
            if(i == shadowLightIndex){
                float shadow;
                if(isShadowAntiAliasingEnabled){
                    shadow  = textureProjOffset(shadowMap, shadowCoord, ivec2(-1, -1));
                    shadow += textureProjOffset(shadowMap, shadowCoord, ivec2(-1,  1));
                    shadow += textureProjOffset(shadowMap, shadowCoord, ivec2( 1,  1));
                    shadow += textureProjOffset(shadowMap, shadowCoord, ivec2( 1, -1));
                    shadow *= 0.25;
                } else {
                    shadow = textureProj(shadowMap, shadowCoord);
                }
                c += shadow * calcDiffuseAndSpecularElements(lights[i]);
            } else {
                c += calcDiffuseAndSpecularElements(lights[i]);
            }
            c += lights[i].ambientIntensity * ambientColor;
        }
        
    }
    color = vec4(c, 1.0);
}
