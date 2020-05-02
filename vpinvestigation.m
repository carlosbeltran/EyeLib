clear all; close all;

% Plot word coordinate frame
Tcam = transl(0,0,1)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-45));

cam = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);

world = SE3();
trplot(world,'frame','0','color','b');

% plot camera
cam.plot_camera();

% Plot points
hold on;
P = mkgrid(2,0.2,'pose',transl(0.7,0.7,0));
plot3(P(1,:),P(2,:),P(3,:));
%figure;

cam.plot(P);
xy = cam.project(P);
xx = xy(1,:);
yy = xy(2,:);

figure;
set(gca,'Ydir','reverse')
p1 = [xx(1),yy(1)];
p2 = [xx(2),yy(2)];
p3 = [xx(3),yy(3)];
p4 = [xx(4),yy(4)];
hold on
plot(p1(1),p1(2),'*r');
plot(p2(1),p2(2),'*r');
plot(p3(1),p3(2),'*y');
plot(p4(1),p4(2),'*y');

line1 = cross([p1,1],[p2,1]);
plot(xx(1:2),yy(1:2),'-r');
line2 = cross([p4,1],[p3,1]);
plot([xx(4);xx(3)],[yy(4);yy(3)],'-r');

line3 = cross([p4,1],[p1,1]);
plot([xx(4);xx(1)],[yy(4);yy(1)],'-y');
line4 = cross([p3,1],[p2,1]);
plot([xx(3);xx(2)],[yy(3);yy(2)],'-y');

vp1 = cross(line1,line2);
vp2 = cross(line3,line4);
vp1 = vp1/vp1(3)
vp2 = vp2/vp2(3)
plot(vp1(1)/vp1(3),vp1(2)/vp1(3),'*r');
plot(vp2(1)/vp2(3),vp2(2)/vp2(3),'*y');

rot = cam.T.R;
rot = rot';
vv1 = cam.K * rot(:,1);
vv1 = vv1/vv1(3)
vv2 = cam.K * rot(:,2);
vv2 = vv2/vv2(3)
vv3 = cam.K * rot(:,3);
vv3 = vv3/vv3(3)
rot
r1 = (inv(cam.K)*vp1')/norm((inv(cam.K)*vp1'))
r2 = (inv(cam.K)*vp2')/norm((inv(cam.K)*vp2'))
r3 = cross(r1,r2)

%% Calculate focal length

vp_x = vv1;
vp_y = vv2;
vp_z = vv3;

temp_1_x = vp_x(1)+vp_y(1);
temp_2_x = vp_y(1)+vp_z(1);
temp_3_x = vp_z(1)+vp_x(1);
temp_1_y = vp_x(2)+vp_y(2);
temp_2_y = vp_y(2)+vp_z(2);
temp_3_y = vp_z(2)+vp_x(2);
temp_xy = vp_x(1)*vp_y(1)+vp_x(2)*vp_y(2);
temp_yz = vp_z(1)*vp_y(1)+vp_z(2)*vp_y(2);
temp_xz = vp_x(1)*vp_z(1)+vp_x(2)*vp_z(2);

%solve for image center ax+by+c=0
syms u_0 v_0;
[final_u, final_v] = solve(-(temp_1_x)*u_0 - (temp_1_y)*v_0 + temp_xy == -(temp_2_x)*u_0 -(temp_2_y)*v_0 + temp_yz,...
-(temp_3_x)*u_0 - (temp_3_y)*v_0 + temp_xz == -(temp_2_x)*u_0 - (temp_2_y)*v_0 + temp_yz);

u = double(final_u);
v = double(final_v);
%focal length solving
eq_f = (u - vp_x(1))*(u - vp_y(1)) + (v - vp_x(2))*(v - vp_y(2));
f = sqrt(-eq_f);
f = double(f);


K = [f      0       u;
     0      f       v;
     0      0       1]   