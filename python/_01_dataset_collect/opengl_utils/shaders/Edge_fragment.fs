#version 450 core

in vec2 texCoord;

out vec4 FragColor;

uniform float delta_x;
uniform float delta_y;

uniform float strong_threshold;

uniform float weak_threshold;
uniform float distance_threshold;

uniform sampler2D normal_map;
uniform sampler2D position_map;

void main()
{
    vec2 sample_pos[8] = vec2[](
        texCoord.st + vec2(-delta_x,  delta_y), // up left
        texCoord.st + vec2( 0,  delta_y), // up
        texCoord.st + vec2( delta_x,  delta_y), // up right
        texCoord.st + vec2(-delta_x,  0),   // left
        texCoord.st + vec2( delta_x,  0),   // right
        texCoord.st + vec2(-delta_x, -delta_y), // bottom left
        texCoord.st + vec2( 0, -delta_y), // bottom
        texCoord.st + vec2( delta_x, -delta_y)  // bottom right
    );
    // solve 
    vec3 n = vec3(texture(normal_map, texCoord).xyz);
    vec3 pos = vec3(texture(position_map, texCoord).xyz);
    float mag = 0.0;
    float dis = 0.0;
    for(int i = 0; i < 8; ++i)
    {
        vec3 dif = vec3(texture(normal_map, sample_pos[i]).xyz) - n;
        float magi = length(dif);
        mag = max(mag, magi);
        if(magi < weak_threshold)
        {
            vec3 pos_dif = vec3(texture(position_map, sample_pos[i]).xyz) - pos;
            dis = max(dis, abs(dot(n, pos_dif)));
        }
    }
    if(mag > strong_threshold || dis > distance_threshold)
        FragColor = vec4(1.0f);
    else
        FragColor = vec4(0.0f);
}