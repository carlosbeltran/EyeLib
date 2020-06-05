clear all; close all;
addpath('./matlab');

% Dependency on Peter Corke Toolboxes (Machine Vision and Robotics)

%% Groundtruth

K_gt = [[403.76110881   0.    288.]; ...
        [  0.    403.76110881 216.]; ...
        [  0.       0.         1. ]];
    
fov_gt        = 71.0;
camera_height = 3.17; % Approximate
pitch_gt      = -30;
yaw_gt        = 151;
roll_gt       = 0.001;
image_width   = 576;
image_height  = 432;

Rot_gt = rotx(-90,'deg')...
         *roty(yaw_gt,'deg')...
         *rotx(pitch_gt,'deg')...
         *rotz(roll_gt,'deg');
% Rot_gt =  roty(yaw_gt,'deg')...
%          *rotx(pitch_gt,'deg')...
%          *rotz(roll_gt,'deg');
     
Rot_gt_inv = Rot_gt';

Tcam = transl(0,0,camera_height)...
       *trotx(-90,'deg')...
       *troty(yaw_gt,'deg')...
       *trotx(pitch_gt,'deg')...
       *trotz(roll_gt,'deg');
   
%vps_gt_ = [[ 1195.99492678  192.67503264]; ...
%          [ 104.98138221   80.65389521]; ...
%          [ 144.38825531   1614.68566071]];
[v1,v2,v3] = computeVanishingPointsviaRandK(Rot_gt_inv,K_gt)
vps_gt = vertcat(h2e(v1)',h2e(v2)',h2e(v3)');
   
cam = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);
world = SE3();
trplot(world,'frame','0','color','b');
cam.plot_camera();

xlim([-1 5]);
ylim([-1 5]);
zlim([-1 10]);

% Get intrinsics matrix K from vanishing points
vpx = vps_gt(1,:);
vpy = vps_gt(2,:);
vpz = vps_gt(3,:);
K   = computeKviaIAC([vpx,1],[vpy,1],[vpz,1]);

% open image
f1 = figure(2);
I = imread('./syntheticdata/people_01.png');
I = insertShape(I,'Line',[vps_gt(1,1) vps_gt(1,2) vps_gt(2,1) vps_gt(2,2)],...
    'LineWidth',2,'Color','blue');
 imshow(I);
 hold on

% Plot vanishing points
plot(vps_gt(1,1),vps_gt(1,2),'*r');
plot(vps_gt(2,1),vps_gt(2,2),'*g');
plot(vps_gt(3,1),vps_gt(3,2),'*b');

% compute horizon line
horizon = cross([vps_gt(1,:),1],[vps_gt(2,:),1]);

%computer roll around z axis
theta_z = atan(-horizon(1)/horizon(2));
roll_deg = rad2deg(theta_z);
%roll_deg = -roll_deg;
roll_gt;

% Get Pitch using the IAC by means of the intrinsics matrix K and the
% vertical vanishg point
pitch_iac = getPitchByIAC(K,vps_gt(3,:))
pitch_gt

%% compute homograpy
Hrot_gt = K_gt ... 
       *rotx(90+pitch_gt,'deg')...
       *inv(K_gt)... 
       *rotz(roll_gt,'deg');

Hrot = K ... 
       *rotx(90-pitch_iac,'deg')... %% Pitch
       *inv(K)... 
       *rotz(-roll_deg,'deg');
   
% compute traslation
% frame_width  = image_width;
% frame_height = image_height;
% 
% tlc = [0, 0];
% trc = [frame_width, 0];
% blc = [0, frame_height];
% brc = [frame_width, frame_height];
% 
% P1 = horzcat(tlc',trc',blc',brc');
% P2 = homtrans(inv(Hrot), P1);
% 
% xmin = min(P2(1,:));
% xmax = max(P2(1,:));
% ymin = min(P2(2,:));
% ymax = max(P2(2,:));
% 
% Tscene = [[1, 0, -xmin]; 
%      [0, 1, -ymin];
%      [0, 0, 1]];
%  
% % Ralign = rotz(20,'deg');
% % Hrot_final = Ralign * Tscene * Hrot;
% Hrot_final = Tscene * Hrot;
% 
% cb_ref = imref2d(size(I))
% cb_translated_ref = cb_ref;
% cb_translated_ref.XWorldLimits(1) = cb_translated_ref.XWorldLimits(1)-500;
% cb_translated_ref.YWorldLimits(1) = cb_translated_ref.YWorldLimits(1)-2000;
% cb_translated_ref.XWorldLimits(2) = cb_translated_ref.XWorldLimits(2)+1500;
% cb_translated_ref.YWorldLimits(2) = cb_translated_ref.YWorldLimits(2)+1500;
% % % % % apply rototranslation to the image plane
% figure;
% tform = projective2d(Hrot_final');
% [imout,trans_ref] = imwarp(I,tform,'OutputView',cb_translated_ref);
% % % figure;
% imshow(imout,trans_ref);
% hold on;

figure(f1);
imshow(I);hold on;

% 
% Just with rotations (no translation no aligment)
pt      = [200;166];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

% Just with rotations (no translation no aligment)
pt      = [200;166];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

% Just with rotations (no translation no aligment)
pt      = [116;207];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

% Just with rotations (no translation no aligment)
pt      = [212;356];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

% Just with rotations (no translation no aligment)
pt      = [328;116];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

% Just with rotations (no translation no aligment)
pt      = [434;166];
pt      = e2h(pt);
dst_pt  = Hrot*pt;
dst_pt  = h2e(dst_pt);
dst_cir = circle(dst_pt, 100);
elip    = inv(Hrot)* e2h(dst_cir);
elip    = h2e(elip);
plot(elip(1,:),elip(2,:),'-g');

