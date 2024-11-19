#version 450 core
out vec3 FragColor;

in vec3 myNormal;

void main()
{    
    // we should convert it to Camera Coordinate System used in Pinhole Model.
        FragColor = normalize(vec3(myNormal.x, -myNormal.y, -myNormal.z));
//     FragColor = vec3(myNormal.z,myNormal.x,myNormal.y);

    //if(FragColor.z > 0.0f)
        //FragColor = -FragColor;
}