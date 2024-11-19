#version 450 core

in vec2 texCoord;

out vec3 normal;

uniform float delta_x;
uniform float delta_y;
uniform sampler2D point_cloud;

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
    vec3 loc = vec3(texture(point_cloud, texCoord).xyz);
    mat2 A = mat2(0.0f);
    vec2 B = vec2(0.0f);
    for(int i = 0; i < 8; ++i)
    {
        vec3 tang = vec3(texture(point_cloud, sample_pos[i]).xyz) - loc;
        A[0][0] += tang.x * tang.x;
        A[0][1] += tang.x * tang.y;
        A[1][0] += tang.x * tang.y;
        A[1][1] += tang.y * tang.y;
        B.x += tang.x * tang.z;
        B.y += tang.y * tang.z;
    }
    if(determinant(A) == 0.0f)
        normal = vec3(0.0f, 0.0f, 0.0f);
    else
    {
        vec2 nxy = inverse(A) * B;
        // note we have already converted coordinate system in position map.
        // here we should not do it again.
        normal = normalize(vec3(nxy, -1.0f));
    }
}


