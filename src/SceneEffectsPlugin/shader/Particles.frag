#version 330

in vec3 position;
in float alpha;

layout (location = 0) out vec4 fragColor;

uniform sampler2D particleTex;

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

uniform vec3 fogColor;
uniform float maxFogDist;
uniform float minFogDist;
uniform bool isFogEnabled = false;

vec3 calcLightingColor(vec3 color, LightInfo light)
{
    if(light.position.w == 0.0){
        // directional light
        return light.intensity * color;
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
        float distance = sqrt(dot(l, l));
        ki *= 1.0 / max(1.0,
                        light.constantAttenuation +
                        distance * light.linearAttenuation +
                        distance * distance * light.quadraticAttenuation);
        
        return ki * light.intensity * color;
    }
}

void main()
{
    vec4 texColor = texture(particleTex, gl_PointCoord);

    vec3 color = texColor.xyz;
    vec3 c = vec3(0.0);
    for(int i=0; i < numLights; ++i){
        c += calcLightingColor(color, lights[i]);
    }

    if(isFogEnabled){
        float dist = abs(position.z);
        float f = (maxFogDist - dist) / (maxFogDist - minFogDist);
        f = clamp(f, 0.0, 1.0);
        c = mix(fogColor, c, f);
    }

    fragColor = vec4(c, texColor.a * alpha);
}
