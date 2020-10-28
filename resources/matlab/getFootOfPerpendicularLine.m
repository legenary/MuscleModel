function foot = getFootOfPerpendicularLine(pt, s, e)
%getFootOfPerpendicularLine gets the foot of perpendicular from a point in
%space to a line, specified by start and end point

dx = e(1) - s(1);
dy = e(2) - s(2);
dz = e(3) - s(3);

u = ((pt(1)-s(1))*dx + (pt(2)-s(2))*dy + (pt(3)-s(3))*dz)...
    /(dx*dx + dy*dy + dz*dz);

foot(1) = s(1) + u*dx;
foot(2) = s(2) + u*dy;
foot(3) = s(3) + u*dz;



end