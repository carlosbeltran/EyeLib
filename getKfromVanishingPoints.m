 
function K = getKfromVanishingPoints(vp_x,vp_y,vp_z)
%GETKFROMVANISHINGPOINTS Summary of this function goes here
%   Get intrinsics matrix from three hortogonal finite vanishing points

temp_1_x = vp_x(1)+vp_y(1);
temp_2_x = vp_y(1)+vp_z(1);
temp_3_x = vp_z(1)+vp_x(1);
temp_1_y = vp_x(2)+vp_y(2);
temp_2_y = vp_y(2)+vp_z(2);
temp_3_y = vp_z(2)+vp_x(2);
temp_xy  = vp_x(1)*vp_y(1)+vp_x(2)*vp_y(2);
temp_yz  = vp_z(1)*vp_y(1)+vp_z(2)*vp_y(2);
temp_xz  = vp_x(1)*vp_z(1)+vp_x(2)*vp_z(2);

%solve for image center ax+by+c=0
syms u_0 v_0;
[final_u, final_v] = solve( ...
-(temp_1_x)*u_0 - (temp_1_y)*v_0 + temp_xy == -(temp_2_x)*u_0 - (temp_2_y)*v_0 + temp_yz,...
-(temp_3_x)*u_0 - (temp_3_y)*v_0 + temp_xz == -(temp_2_x)*u_0 - (temp_2_y)*v_0 + temp_yz...
);

u = double(final_u);
v = double(final_v);

%focal length solving
eq_f = (u - vp_x(1))*(u - vp_y(1)) + (v - vp_x(2))*(v - vp_y(2));
f = sqrt(-eq_f);
f = double(f);


K = [f      0       u;
     0      f       v;
     0      0       1];
 
end

