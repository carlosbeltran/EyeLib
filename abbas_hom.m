clear all; close all;

%% Groundtruth
% % vps = [[1195.99492678  192.67503264]; ...
% %  [ 104.98138221   80.65389521]; ...
% %  [ 144.38825531 1614.68566071]];
% % 
% % K = [[403.76110881   0. 288.        ]; ...
% %  [  0.    403.76110881 216.        ]; ...
% %  [  0.       0.           1.        ]];
% % fov = 71.0;
% % camera_height = 7.692022323608398
% % pitch = -16.022058486938477;
% % yaw = 24.999956130981445;
% % roll = -5.862370014190674;
% % image_width = 576;
% % image_height = 432;

vps = [[-3177.8156919   -176.18515848]; ...
 [  319.23931061     4.49912707]; ...
 [  249.04268972   621.61837748]];
K = [[294.65329529   0.         270.        ]; ...
 [  0.         294.65329529 216.        ]; ...
 [  0.           0.           1.        ]];

camera_height = 19.580541610717773;
pitch = -35.95946502685547;
yaw = -6.000004291534424;
roll = -2.957702875137329;
image_width = 540;
image_height = 432;


% open image
%I = imread('town_1_frame_1268.png');
I = imread('town_1_frame_2098.png');
imshow(I);
hold on

% Plot vanishing points
plot(vps(1,1),vps(1,2),'*r');
plot(vps(2,1),vps(2,2),'*g');
plot(vps(3,1),vps(3,2),'*b');

% compute horizon line
line1 = cross([vps(1,:),1],[vps(2,:),1]);

%computer roll around z axis
theta_z = atan(-line1(1)/line1(2));
roll_deg = rad2deg(theta_z)
roll

%compute rotation matrix on z axis

Rz = [ cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];

% apply rotation
tform = projective2d(Rz);
imout = imwarp(I,tform);
figure;
%imshow(imout,[],'InitialMagnification','fit');
imshow(imout);

% compute x rotation (tilt)
theta_x = (pi/2.0) - atan(norm([vps(3,:)])/K(1));
tild_deg = rad2deg(theta_x)
pitch
