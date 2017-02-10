#version 400

layout (location = 0) in vec3 vertexInitVel;
layout (location = 1) in float startTime;

out float transparency;

uniform float time;
uniform vec3 gravity = vec3(0.0,-0.05,0.0);
uniform float particleLifeTime;

uniform mat4 MVP;

void main()
{
    vec3 pos = vec3(0.0);
    transparency = 0.0;

    if( time > startTime ) {
        float t = time - startTime;

        if( t < particleLifeTime ) {
            pos = vertexInitVel * t + gravity * t * t;
            transparency = 1.0 - t / particleLifeTime;
        }
    }

    gl_Position = MVP * vec4(pos, 1.0);
}
