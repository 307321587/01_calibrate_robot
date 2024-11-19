#version 330 core
in vec3 viewpos;

out vec3 FragColor;
void main()
{
    // we should convert it to Camera Coordinate System used ing Pinhole Model.
    FragColor = vec3(viewpos.x, -viewpos.y, -viewpos.z);
};