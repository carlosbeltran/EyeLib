clear all; close all;
addpath('./matlab');

% Dependency on Peter Corke Toolboxes (Machine Vision and Robotics)

%% Groundtruth
vps_gt = [[ 1195.99492678  192.67503264]; ...
          [ 104.98138221   80.65389521]; ...
          [ 144.38825531   1614.68566071]];

K_gt = [[403.76110881   0.    288.]; ...
        [  0.    403.76110881 216.]; ...
        [  0.       0.         1. ]];
    
fov_gt        = 71.0;
camera_height = 7.692022323608398;
pitch_gt      = -16.022058486938477;
yaw_gt        = 24.999956130981445;
roll_gt       = -5.862370014190674;
image_width   = 576;
image_height  = 432;

Rot_gt = rotx(-90,'deg')...
         *roty(yaw_gt,'deg')...
         *rotx(pitch_gt,'deg')...
         *rotz(roll_gt,'deg');
Rot_gt_inv = Rot_gt';

Tcam = transl(0,0,camera_height)...
       *trotx(-90,'deg')...
       *troty(yaw_gt,'deg')...
       *trotx(pitch_gt,'deg')...
       *trotz(roll_gt,'deg');
   
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
%K = getKfromVanishingPoints(vpx,vpy,vpz);
K = computeKviaIAC([vpx,1],[vpy,1],[vpz,1]);

%check vanishing points from rotation matrix
vvv1 = K_gt*Rot_gt_inv(:,1);
vvv1 = h2e(vvv1)'
gt_vp1 = vps_gt(1,:)

vvv2 = K_gt*Rot_gt_inv(:,2);
vvv2 = h2e(vvv2)'
gt_vp2 = vps_gt(2,:)

vvv3 = K_gt*Rot_gt_inv(:,3);
vvv3 = h2e(vvv3)
gt_vp3 = vps_gt(3,:)

% open image
I = imread('./syntheticdata/town_1_frame_1268.png');
I = insertShape(I,'Line',[vps_gt(1,1) vps_gt(1,2) vps_gt(2,1) vps_gt(2,2)],...
    'LineWidth',2,'Color','blue');
 imshow(I);
% hold on

% Plot vanishing points
% plot(vps_gt(1,1),vps_gt(1,2),'*r');
% plot(vps_gt(2,1),vps_gt(2,2),'*g');
% plot(vps_gt(3,1),vps_gt(3,2),'*b');

% compute horizon line
horizon = cross([vps_gt(1,:),1],[vps_gt(2,:),1]);

%computer roll around z axis
theta_z = atan(-horizon(1)/horizon(2));
roll_deg = rad2deg(theta_z);
roll_gt;

% %Rotation matrix on z axis
% Rz = [ cos(theta_z) -sin(theta_z) 0;
%     sin(theta_z) cos(theta_z) 0;
%     0 0 1];

% % compute x rotation (tilt) [ this doesn't seem to work, error in stimated
% % angle ]
% theta_x = (pi/2.0) - atan(norm([vps_gt(3,:)])/K_gt(1));
% tild_deg = rad2deg(theta_x);
% pitch_gt;

% Get Pitch using the IAC by means of the intrinsics matrix K and the
% vertical vanishg point
pitch_iac = getPitchByIAC(K,vps_gt(3,:));

% Check the rotation matrix using the vanishing points
r1 = (inv(K_gt)*[vps_gt(1,:),1]')/norm(inv(K_gt)*[vps_gt(1,:),1]'); 
r2 = (inv(K_gt)*[vps_gt(2,:),1]')/norm(inv(K_gt)*[vps_gt(2,:),1]');
r3 = cross(r1,r2);
RotM_2 = [r1,r2,r3]'

% Rot_gt
% cam.T


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
frame_width  = image_width;
frame_height = image_height;

tlc = [0, 0];
trc = [frame_width, 0];
blc = [0, frame_height];
brc = [frame_width, frame_height];

P1 = horzcat(tlc',trc',blc',brc');
P2 = homtrans(inv(Hrot), P1);

xmin = min(P2(1,:));
xmax = max(P2(1,:));
ymin = min(P2(2,:));
ymax = max(P2(2,:));

Tscene = [[1, 0, -xmin]; 
     [0, 1, -ymin];
     [0, 0, 1]];
 
Ralign = rotz(20,'deg');
Hrot_final = Ralign * Tscene * Hrot;

cb_ref = imref2d(size(I))
cb_translated_ref = cb_ref;
cb_translated_ref.XWorldLimits(1) = cb_translated_ref.XWorldLimits(1)-500;
cb_translated_ref.YWorldLimits(1) = cb_translated_ref.YWorldLimits(1)-2000;
cb_translated_ref.XWorldLimits(2) = cb_translated_ref.XWorldLimits(2)+1500;
cb_translated_ref.YWorldLimits(2) = cb_translated_ref.YWorldLimits(2)+1500;
% % % % apply rototranslation to the image plane
figure;
tform = projective2d(Hrot_final');
[imout,trans_ref] = imwarp(I,tform,'OutputView',cb_translated_ref);
% % figure;
imshow(imout,trans_ref);
hold on;
% plot(P2(1,:),P2(2,:),'*r');
