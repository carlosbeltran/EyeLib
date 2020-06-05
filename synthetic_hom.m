clear all; close all;
addpath('./external');
addpath('./matlab');

% Plot word coordinate frame
%Tcam = transl(0,0,3)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-45));
Tcam = transl(0,0,0.5)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-5));
%Tcam = transl(0,0,8)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-60));


cam = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);

world = SE3();
trplot(world,'frame','0','color','b');

% plot camera
cam.plot_camera();
hold on;

%Add a poles
Zt=1.8;

P = [ 3 3; 3 3; 0 Zt];
plot3(P(1,:),P(2,:),P(3,:),'LineWidth',7); 
cam.plot(P);
B = P(:,1);
T = P(:,2);
xy = cam.project(P);
b1 = [xy(:,1);1];
t1 = [xy(:,2);1];
% create ground truth circle in 3D
gt_circle = circle(B, 0.5);
% get the ground truth circle projection in 2D
gt_elipse = cam.project(gt_circle);

P = [ 1 1; 3 3; 0 Zt];
plot3(P(1,:),P(2,:),P(3,:),'LineWidth',7); 
cam.plot(P);
B = P(:,1);
T = P(:,2);
xy = cam.project(P);
b2 = [xy(:,1);1];
t2 = [xy(:,2);1];

xlim([-1 5]);
ylim([-1 5]);
zlim([-0.1 5.5]);

% Plot points
P = mkgrid(2,1,'pose',transl(2,2,0));
plot3(P(1,:),P(2,:),P(3,:));
%figure;

cam.plot(P);
xy = cam.project(P);
xx = xy(1,:);
yy = xy(2,:);

figure;
set(gca,'Ydir','reverse')
%%[1280 1024]
xlim([0 1280]);
ylim([0 1024]);

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
%plot(vp1(1)/vp1(3),vp1(2)/vp1(3),'*r');
%plot(vp2(1)/vp2(3),vp2(2)/vp2(3),'*y');

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

%Other K
[f,u0,v0,R] = camCalibFromVP(horzcat(vv1(1:2),vv2(1:2),vv3(1:2)));
Kother = [f , 0 , u0;
          0 , f , v0;
          0 , 0 ,  1]
      
% compute horizon line
%horizon = cross([vps_gt(1,:),1],[vps_gt(2,:),1]);
horizon = cross(vv1',vv2');

%computer roll around z axis
theta_z  = atan(-horizon(1)/horizon(2));
roll_deg = rad2deg(theta_z);

% Get Pitch using the IAC by means of the intrinsics matrix K and the
% vertical vanishg point
%pitch_iac = getPitchByIAC(K,vps_gt(3,:))
pitch_iac = getPitchByIAC(K,vv3(1:2)');

%% compute homograpy
Hrot = K ... 
       *rotx(90-pitch_iac,'deg')... %% Pitch
       *inv(K)... 
       *rotz(-roll_deg,'deg');
   
figure(3);hold on;
% 
% Just with rotations (no translation no aligment)
%pt      = [200;166];
pt = b1(1:2);
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 250);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-r');

plot(gt_elipse(1,:),gt_elipse(2,:),'-g');

