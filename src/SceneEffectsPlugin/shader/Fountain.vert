#version 330

layout (location = 0) in vec3 vertexInitVel;
layout (location = 1) in float offsetTime;

out vec3 position;
out float alpha;

uniform float time;
uniform float lifeTime;
uniform vec3 accel = vec3(0.0, 0.0, -0.2);

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform float pointSize;
uniform float angle2pixels;

void main()
{
    vec3 pos = vec3(0.0);
    alpha = 0.0;
    float t = time - offsetTime;
    if(t > 0){
        t = mod(t, lifeTime);
        pos = vertexInitVel * t + accel * t * t;
        alpha = 1.0 - t / lifeTime;
    }

    vec4 lpos = modelViewMatrix * vec4(pos, 1.0);
    position = lpos.xyz;
    gl_Position = projectionMatrix * lpos;

    if(pointSize >= 0.0){ // Perspective
        float d = max(1.0e-6, sqrt(dot(position, position)));
        float angle = asin(pointSize / d);
        gl_PointSize = angle * angle2pixels;

    } else {  // Orthographic
        gl_PointSize = -pointSize;
    }
}
