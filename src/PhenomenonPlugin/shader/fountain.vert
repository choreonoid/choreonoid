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
uniform mat4 projectionMatrix;
uniform float pointSize;
uniform float angle2pixels;

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

    vec4 lpos = modelViewMatrix * vec4(pos, 1.0);
    position = vec3(lpos);
    gl_Position = projectionMatrix * lpos;

    if(pointSize >= 0.0){ // Perspective
        float d = max(1.0e-6, sqrt(dot(position, position)));
        float angle = asin(pointSize / d);
        gl_PointSize = angle * angle2pixels;

    } else {  // Orthographic
        gl_PointSize = -pointSize;
    }
}
