#version 330

uniform bool colorPerVertex;
uniform vec3 color;

in vec3 colorV;

layout(location = 0) out vec4 fragColor;

void main() {
    if(colorPerVertex){
        fragColor = vec4(colorV, 1.0);
    } else {
        fragColor = vec4(color, 1.0);
    }
}
