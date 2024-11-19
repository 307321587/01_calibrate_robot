#version 450 core
out vec4 FragColor;

uniform bool TLess_flag;
uniform vec3 bottom_color;
uniform sampler2D texture_diffuse;

in vec2 TexCoords;

void main()
{    
    if(TLess_flag)
        FragColor = vec4(bottom_color, 1.0f);
    else
        FragColor = texture(texture_diffuse, TexCoords);
}