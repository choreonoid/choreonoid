#version 330

uniform vec3 PickingID;

layout(location = 0) out vec4 FragColor;

void main() {
    FragColor = vec4(PickingID, 1.0);
}
