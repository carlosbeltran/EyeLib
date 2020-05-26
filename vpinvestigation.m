clear all; close all;
addpath('./external');
addpath('./matlab');

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

rot = cam.T.R; % rotaton part of the "pose" of the camera wRc
rot = rot';% getting cRw
cRw = rot;
[vv1,vv2,vv3] = computeVanishingPointsviaRandK(cRw,cam.K);
vv1
vv2
vv3

cRw_gt = cam.T.R'
Rest   = computeRviaVanishingPointsandK(cam.K,vv1,vv2)

%% Calculate focal length

K = computeKviaIAC(vv1,vv2,vv3)
cam.K
assert(isalmost(K,cam.K,1e-10),"Comparison of K matrices")

   