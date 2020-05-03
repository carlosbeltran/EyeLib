clear all; close all;

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

%Rot_gt = rpy2r(roll_gt,pitch_gt,yaw_gt,'deg','camera');
Rot_gt = rpy2r(roll_gt,pitch_gt,yaw_gt,'deg','yxz');
%Rot_gt_roll_pitch = rpy2r(roll_gt,pitch_gt,0,'deg','yxz')
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

%check vanishing points from rotation matrix
vvv1 = K_gt*Rot_gt_inv(:,1);
vvv1 = vvv1/vvv1(3);
vvv1'
gt_vp1 = vps_gt(1,:)

vvv2 = K_gt*Rot_gt_inv(:,2);
vvv2 = vvv2/vvv2(3);
vvv2'
gt_vp2 = vps_gt(2,:)

vvv3 = K_gt*Rot_gt_inv(:,3);
vvv3 = vvv3/vvv3(3);
vvv3'
gt_vp3 = vps_gt(3,:)

% open image
I = imread('town_1_frame_1268.png');
%I = imread('town_1_frame_2098.png');
% imshow(I);
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

%Rotation matrix on z axis
Rz = [ cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];

% compute x rotation (tilt) [ this doesn't seem to work, error in stimated
% angle ]
theta_x = (pi/2.0) - atan(norm([vps_gt(3,:)])/K_gt(1));
tild_deg = rad2deg(theta_x);
pitch_gt;

% instead trying to get the pitch angle from the Image of The Absolute Conic (IAC)
% using the camera center vector and the vertical vanishing point (vps3_gt)
% check page 210 of Hartley and Zisserman " Multiple View Geometry in
% computer vision" [second edition]
x1  = K_gt(:,3); % Image centre (first image point)
iac = inv(K_gt*K_gt'); %Image of the absolute conic
x2  = [vps_gt(3,:),1]'; % Vanishing point (second image point)
costheta = (x1'*iac*x2)/(sqrt(x1'*iac*x1)*sqrt(x2'*iac*x2));
pitch_iac = rad2deg(pi/2 - acos(costheta));

theta_x = pi/2 - acos(costheta);

Rx = [ 1  0 0;
    0 cos(theta_x) -sin(theta_x);
    0 sin(theta_x) cos(theta_x)];

%%%RotM = Rx*Rz

% Compute directly rotation matrix columns
% It doesn't seem to work
% wrongly compute vanishing points?
r1 = (inv(K_gt)*[vps_gt(1,:),1]')/norm(inv(K_gt)*[vps_gt(1,:),1]'); 
r2 = (inv(K_gt)*[vps_gt(2,:),1]')/norm(inv(K_gt)*[vps_gt(2,:),1]');
r3 = cross(r1,r2);
RotM_2 = [r1,r2,r3]'

Rot_gt
cam.T
%Rot_gt_roll_pitch


% % % %Compute Homography??
% % %Hrot = K_gt*inv(Rx_gt)*inv(K_gt)*inv(Rz_gt);
% % Hrot = K_gt*Rx*inv(K_gt)*Rz;
% % 
% % %Hrot = inv(Rz_gt) * inv(Rx_gt);
% % 
% % % % apply rotation to the image plane
% % tform = projective2d(Hrot');
% % imout = imwarp(I,tform);
% % figure;
% % imshow(imout);
