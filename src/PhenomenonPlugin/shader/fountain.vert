#version 330

layout (location = 0) in vec3 vertexInitVel;
layout (location = 1) in float offsetTime;

out float transparency;
out vec3 position;

uniform float time;
uniform float lifeTime;
uniform float cycleTime;
uniform vec3 gravity = vec3(0.0, 0.0, -0.05);

uniform mat4 modelViewMatrix;
uniform mat4 MVP;

void main()
{
    vec3 pos = vec3(0.0);
    transparency = 0.0;
    float t = time - offsetTime;
    if(t > 0){
        t = mod(t, cycleTime);
        if(t < lifeTime){
            pos = vertexInitVel * t + gravity * t * t;
            transparency = 1.0 - t / lifeTime;
        }
    }

    vec4 pos4 = vec4(pos, 1.0);
    position = vec3(modelViewMatrix * pos4);
    gl_Position = MVP * pos4;
    gl_PointSize = 12.0;
}
