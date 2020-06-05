clear all; close all;
addpath('./external');
addpath('./matlab');

f1 = figure(1);

I = imread('..\2a_background.jpg');
imshow(I);

imsizex = 2048;
imsizey = 1536;

%[x,y] = getpts();

pts = [1108.8 1141.4;
1153.3 917.17;
1643 952.15;
1690.7 1181.1;
771.68 1152.5;
717.62 758.16;
1455.4 1222.5;
1534.9 826.54];

xx = pts(:,1);
yy = pts(:,2);

hold on;

plot(xx,yy,'*g');

p1 = [pts(1,:),1];
p2 = [pts(2,:),1];
p3 = [pts(3,:),1];
p4 = [pts(4,:),1];
p5 = [pts(5,:),1];
p6 = [pts(6,:),1];
p7 = [pts(7,:),1];
p8 = [pts(8,:),1];

% X direction vanising point
line1_4 = cross(p1,p4);
plot([xx(1);xx(4)],[yy(1);yy(4)],'-r','LineWidth',5);
line2_3 = cross(p2,p3);
plot([xx(2);xx(3)],[yy(2);yy(3)],'-r','LineWidth',5);

vp1 = cross(line1_4,line2_3);
vp1 = vp1/vp1(3);
v1 = h2e(vp1);

% Y direction vanising point
line1_2 = cross(p1,p2);
plot([xx(1);xx(2)],[yy(1);yy(2)],'-g','LineWidth',5);
line4_3 = cross(p4,p3);
plot([xx(4);xx(3)],[yy(4);yy(3)],'-g','LineWidth',5);

vp2 = cross(line1_2,line4_3);
vp2 = vp2/vp2(3);
v2 = h2e(vp2);

% Z direction vanising point
line5_6 = cross(p5,p6);
plot([xx(5);xx(6)],[yy(5);yy(6)],'-b','LineWidth',5);
line7_8 = cross(p7,p8);
plot([xx(7);xx(8)],[yy(7);yy(8)],'-b','LineWidth',5);

vp3 = cross(line5_6,line7_8);
vp3 = vp3/vp3(3);
v3 = h2e(vp3);

%Compute calibration matrix
%
%K   = getKfromVanishingPoints(v1',v2',v3');
[f,u0,v0,R] = camCalibFromVP(horzcat(v1,v2,v3));
K = [f , 0 , u0;
     0 , f , -v0;
     0 , 0 ,  1];
 
Rest   = computeRviaVanishingPointsandK(K,vp1',vp2')
 
% Try ploting a camera
figure(2);
Camheigh_guess = 3 ; % 3 meters
Tcam = transl(0,0,Camheigh_guess)...
       *r2t(Rest');
cam = CentralCamera('focal', f, ...
     'resolution',[2048 1536],'centre',[u0 v0],'pose',Tcam);
world = SE3();
trplot(world,'frame','0','color','b');
cam.plot_camera();

xlim([-1 5]);
ylim([-1 5]);
zlim([-1 10]);

% Getting Abbas transform
vpp3 = vp3/vp3(3);
pitch_iac = getPitchByIAC(K,vpp3(1:2))

% compute horizon line
horizon = cross(vp1,vp2);
%computer roll around z axis
theta_z = atan(-horizon(1)/horizon(2));
roll_deg = rad2deg(theta_z);

Hrot = K ... 
       *rotx(90-pitch_iac,'deg')... %% Pitch
       *inv(K)... 
       *rotz(-roll_deg,'deg');
   
figure(3);
imshow(I);hold on;

% reproject a pole base location
pt      = [1345;885];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 300);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g','LineWidth',5);

plot(u0,-v0,'*r');


% % Try ploting bird eye camera
% figure(4); 
% Camheigh_guess = 3 ; % 3 meters
% Tcam2 = transl(0,0,Camheigh_guess)...
%        *r2t(Hrot);
% cam2 = CentralCamera('focal', f, ...
%      'resolution',[2048 1536],'centre',[u0 v0],'pose',Tcam2,'color','r');
% world2 = SE3();
% trplot(world2,'frame','0','color','b');
% cam2.plot_camera();
% 
% xlim([-1 5]);
% ylim([-1 5]);
% zlim([-1 5]);
   