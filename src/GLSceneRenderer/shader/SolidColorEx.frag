#version 330

uniform bool colorPerVertex;
uniform vec3 color;
uniform float alpha;

in VertexData {
    vec3 color;
} inData;

layout(location = 0) out vec4 fragColor;

void main()
{
    if(colorPerVertex){
        fragColor = vec4(inData.color, alpha);
    } else {
        fragColor = vec4(color, alpha);
    }
}
