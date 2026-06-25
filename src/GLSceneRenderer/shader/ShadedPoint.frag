#version 330

#define MAX_NUM_LIGHTS 20

in VertexData {
    vec3 position;
    vec3 normal;
    vec3 colorV;
} inData;

uniform vec3 diffuseColor;
uniform float ambientIntensity;
uniform vec3 specularColor;
uniform vec3 emissionColor;
uniform float specularExponent;
uniform float alpha = 1.0;

uniform int numLights;

struct LightInfo {
    vec4 position;
    vec3 intensity;
    vec3 ambientIntensity;
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    float cutoffAngle;
    float beamWidth;
    float cutoffExponent;
    vec3 direction;
};

uniform LightInfo lights[MAX_NUM_LIGHTS];

uniform bool isVertexColorEnabled;
uniform vec3 highlightColor = vec3(1.0, 1.0, 1.0);
uniform bool isHighlightEnabled = false;
uniform vec3 fogColor;
uniform float maxFogDist;
uniform float minFogDist;
uniform bool isFogEnabled = false;

layout(location = 0) out vec4 color4;

vec3 safeNormalize(vec3 v)
{
    float l2 = dot(v, v);
    return l2 > 1.0e-12 ? v * inversesqrt(l2) : vec3(0.0, 0.0, 1.0);
}

vec3 calcDiffuseAndSpecularElements(LightInfo light, vec3 baseColor, vec3 normal)
{
    vec3 s;
    float attenuation = 1.0;

    if(light.position.w == 0.0){
        s = safeNormalize(vec3(light.position));
    } else {
        vec3 l = vec3(light.position) - inData.position;
        s = safeNormalize(l);

        if(light.cutoffAngle != 0.0){
            vec3 direction = safeNormalize(light.direction);
            float sd = dot(-s, direction);
            float angle = acos(clamp(sd, -1.0, 1.0));
            if(angle >= light.cutoffAngle){
                return vec3(0.0);
            } else if(angle > light.beamWidth){
                float t = (light.cutoffAngle - angle) / (light.cutoffAngle - light.beamWidth);
                attenuation *= pow(t, light.cutoffExponent);
            }
        }

        float distance = length(l);
        attenuation *= 1.0 / max(
            1.0,
            light.constantAttenuation +
            distance * light.linearAttenuation +
            distance * distance * light.quadraticAttenuation);
    }

    float diffuse = max(dot(s, normal), 0.0);
    vec3 spec = vec3(0.0);
    if(!isHighlightEnabled && diffuse > 0.0){
        vec3 v = safeNormalize(-inData.position);
        vec3 h = safeNormalize(v + s);
        spec = specularColor * pow(max(dot(h, normal), 1.0e-6), specularExponent);
    }
    return attenuation * light.intensity * (baseColor * diffuse + spec);
}

void main()
{
    vec2 point = gl_PointCoord * 2.0 - vec2(1.0);
    if(dot(point, point) > 1.0){
        discard;
    }

    vec3 baseColor = diffuseColor;
    if(isVertexColorEnabled){
        baseColor *= inData.colorV;
    }

    if(isHighlightEnabled){
        float maxComponent = max(max(baseColor.r, baseColor.g), baseColor.b);
        baseColor = highlightColor * (0.5 + maxComponent * 0.5);
    }

    vec3 normal = safeNormalize(inData.normal);
    vec3 color = emissionColor;
    for(int i=0; i < numLights; ++i){
        color += lights[i].ambientIntensity * ambientIntensity * baseColor;
        color += calcDiffuseAndSpecularElements(lights[i], baseColor, normal);
    }

    color4 = vec4(color, alpha);

    if(isFogEnabled){
        float dist = abs(inData.position.z);
        float f = (maxFogDist - dist) / (maxFogDist - minFogDist);
        f = clamp(f, 0.0, 1.0);
        color4 = mix(vec4(fogColor, 1.0), color4, f);
    }
}
