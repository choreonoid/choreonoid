#version 330

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

out VertexData {
    vec3 color;
} outData;

in VertexData {
    vec3 color;
} inData[];

uniform float lineWidth = 5.0;
uniform vec2 viewportSize;

void main()
{
    /*
    // Normalized device coordinate
    vec3 p1 = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
    vec3 p2 = gl_in[1].gl_Position.xyz / gl_in[1].gl_Position.w;
    
    vec2 dir = normalize((p2.xy - p1.xy) * viewportSize);
    vec2 normal = vec2(-dir.y, dir.x);
    vec3 offset = vec3(normal * lineWidth * 0.5 / viewportSize * 2.0, 0.0);

    gl_Position = vec4((p1 + offset) * gl_in[0].gl_Position.w, gl_in[0].gl_Position.w);
    outData.color = inData[0].color;
    EmitVertex(); 
    gl_Position = vec4((p1 - offset) * gl_in[0].gl_Position.w, gl_in[0].gl_Position.w);
    outData.color = inData[0].color;
    EmitVertex();
    gl_Position = vec4((p2 + offset) * gl_in[1].gl_Position.w, gl_in[1].gl_Position.w);
    outData.color = inData[1].color;
    EmitVertex();
    gl_Position = vec4((p2 - offset) * gl_in[1].gl_Position.w, gl_in[1].gl_Position.w);
    outData.color = inData[1].color;
    EmitVertex();
    */

    // Optimized version
    vec4 p1 = gl_in[0].gl_Position;
    vec4 p2 = gl_in[1].gl_Position;
    vec2 dir = normalize((p2.xy - p1.xy) * viewportSize);
    vec2 offset = vec2(-dir.y, dir.x) * lineWidth / viewportSize;

    gl_Position = p1 + vec4(offset.xy * p1.w, 0.0, 0.0);
    outData.color = inData[0].color;
    EmitVertex();
    gl_Position = p1 - vec4(offset.xy * p1.w, 0.0, 0.0);
    outData.color = inData[0].color;
    EmitVertex();
    gl_Position = p2 + vec4(offset.xy * p2.w, 0.0, 0.0);
    outData.color = inData[1].color;
    EmitVertex();
    gl_Position = p2 - vec4(offset.xy * p2.w, 0.0, 0.0);
    outData.color = inData[1].color;
    EmitVertex();

    EndPrimitive();
}
