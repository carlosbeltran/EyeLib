clear all; close all;

%% Groundtruth
% vps_gt = [[1195.99492678  192.67503264]; ...
%  [ 104.98138221   80.65389521]; ...
%  [ 144.38825531 1614.68566071]];
% 
% K_gt = [[403.76110881   0. 288.        ]; ...
%  [  0.    403.76110881 216.        ]; ...
%  [  0.       0.           1.        ]];
% fov_gt = 71.0;
% camera_height = 7.692022323608398
% pitch_gt = -16.022058486938477;
% yaw_gt = 24.999956130981445;
% roll_gt = -5.862370014190674;
% image_width = 576;
% image_height = 432;

vps_gt = [[-3177.8156919   -176.18515848]; ...
 [  319.23931061     4.49912707]; ...
 [  249.04268972   621.61837748]];
K_gt = [[294.65329529   0.         270.        ]; ...
 [  0.         294.65329529 216.        ]; ...
 [  0.           0.           1.        ]];

camera_height = 19.580541610717773;
pitch_gt = -35.95946502685547;
yaw_gt = -6.000004291534424;
roll_gt = -2.957702875137329;
image_width = 540;
image_height = 432;


% open image
% I = imread('town_1_frame_1268.png');
I = imread('town_1_frame_2098.png');
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
roll_deg = rad2deg(theta_z)
roll_gt

%Rotation matrix on z axis
Rz = [ cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];

% apply rotation
tform = projective2d(Rz);
imout = imwarp(I,tform);
figure;
imshow(imout);

% compute x rotation (tilt) [ this doesn't seem to work, error in stimated
% angle ]
theta_x = (pi/2.0) - atan(norm([vps_gt(3,:)])/K_gt(1));
tild_deg = rad2deg(theta_x)
pitch_gt

% instead trying to get the pitch angle from the Image of The Absolute Conic (IAC)
% using the camera center vector and the vertical vanishing point (vps3_gt)
% check page 210 of Hartley and Zisserman " Multiple View Geometry in
% computer vision" [second edition]
x1  = K_gt(:,3); % Image centre (first image point)
iac = inv(K_gt*K_gt'); %Image of the absolute conic
x2  = [vps_gt(3,:),1]'; % Vanishing point (second image point)
costheta = (x1'*iac*x2)/(sqrt(x1'*iac*x1)*sqrt(x2'*iac*x2));
pitch_iac = rad2deg(pi/2 - acos(costheta))

