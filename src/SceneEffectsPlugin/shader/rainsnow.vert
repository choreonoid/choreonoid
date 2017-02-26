#version 330

layout (location = 0) in vec3 vertexInitPos;
layout (location = 1) in float offsetTime;

out vec3 position;
out float alpha;

uniform float time;
uniform float lifeTime;
uniform vec3 velocity;

uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform float pointSize;
uniform float angle2pixels;

void main()
{
    vec3 pos = vertexInitPos;
    alpha = 0.0;
    float t = time - offsetTime;
    //if(t > 0){
    if(true){
        t = mod(t, lifeTime);
        pos = vertexInitPos + velocity * t;
        alpha = 1.0;
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
