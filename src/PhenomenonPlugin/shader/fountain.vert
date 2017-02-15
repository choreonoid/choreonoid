#version 330

layout (location = 0) in vec3 vertexInitVel;
layout (location = 1) in float startTime;

out float transparency;

uniform float time;
uniform float lifeTime;
uniform vec3 gravity = vec3(0.0, 0.0, -0.05);

uniform mat4 MVP;

void main()
{
    vec3 pos = vec3(0.0);
    transparency = 0.0;

    if(time > startTime) {
        float t = time - startTime;

        if(t < lifeTime) {
            pos = vertexInitVel * t + gravity * t * t;
            transparency = 1.0 - t / lifeTime;
        }
    }

    gl_Position = MVP * vec4(pos, 1.0);
    gl_PointSize = 10.0;
}
