#version 450 core

in float depth;
out float FragColor;

void main()
{             
    FragColor = depth;
}