function [ f ] = function_inside_outside( xw, yw, zw, px, py, pz,...
                                          a1, a2, a3, Rot, e1, e2)
                                      
%descrition: function to determine the position of a point in relation to  
%            the superellipsoid (f < 1: inside; f = 1: on the surface; 
%            f > 1: outside)
%inputs: xw, yw, zw (arm point position)
%        px, py, pz (center of the superellipsoid)
%        a1, a2, a3 (superellipsoid size)
%        Rot (superellipsoid RPY matrix)
%        e1, e2 (parameters of the shape)
%output: f (function inside-outside)
%author: Sara Sá (a68523)

    nx = Rot(1,1); ny = Rot(2,1); nz = Rot(3,1);
    ox = Rot(1,2); oy = Rot(2,2); oz = Rot(3,2);
    ax = Rot(1,3); ay = Rot(2,3); az = Rot(3,3);

    f = (((nx*xw + ny*yw + nz*zw - px*nx - py*ny - pz*nz)/a1).^(2/e2) + ...
        ((ox*xw + oy*yw + oz*zw - px*ox - py*oy - pz*oz)/a2).^(2/e2)).^(e2/e1) + ...
        ((ax*xw + ay*yw + az*zw - px*ax - py*ay - pz*az)/a3).^(2/e1);
end

