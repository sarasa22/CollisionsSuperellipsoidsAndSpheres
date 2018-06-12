function [ Rot ] = RPY( roll, pitch, yaw )

%description: function to compute the RPY matrix
%inputs: roll (around X axis)
%        pitch (around Y axis)
%        yaw (around Z axis)
%output: Rot (RPY matrix)
%author: Sara Sá (a68523)

    Rot(1,1) = cos(roll) * cos(pitch);
    Rot(1,2) = cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw);
    Rot(1,3) = cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw);
    Rot(2,1) = sin(roll) * cos(pitch);
    Rot(2,2) = sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw);
    Rot(2,3) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
    Rot(3,1) = - sin(pitch);
    Rot(3,2) = cos(pitch) * sin(yaw);
    Rot(3,3) = cos(pitch) * cos(yaw);

end

