#version 330

uniform vec3 pickingID;

layout(location = 0) out vec4 color;

void main() {
    color = vec4(pickingID, 1.0);
}
