#version 130

in vec2 position;
uniform mat4 viewTransform;

void main() {
    gl_Position = viewTransform * vec4(position, 1, 1);
}
