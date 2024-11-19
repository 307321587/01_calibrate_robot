#version 330 core
layout(location=0) in vec3 pos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 viewpos;
void main(){
    vec4 vph=view*model*vec4(pos,1.0);
    viewpos = vec3(vph.x/vph.w, vph.y/vph.w, vph.z/vph.w);
    gl_Position = projection * vph;
}