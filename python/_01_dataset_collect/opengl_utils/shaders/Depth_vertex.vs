#version 450 core
layout (location = 0) in vec3 pos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out float depth;

void main()
{   
    vec4 viewpos = view * model * vec4(pos, 1.0);
    depth = -viewpos.z;
    gl_Position = projection * viewpos;
}