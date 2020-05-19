clear all; close all;

% Plot word coordinate frame
Tcam = transl(0,0,4)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-35));

cam = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);

world = SE3();
trplot(world,'frame','0','color','b');

xlim([-1 5]);
ylim([-1 5]);
zlim([-1 5]);

% plot camera
cam.plot_camera();

% Plot points
hold on;
%%P = mkgrid(2,0.2,'pose',transl(0.7,0.7,0));
P = mkcube(1.8,'pose',transl([4,4,0.9]));
P = P(:,[1,5]);
plot3(P(1,:),P(2,:),P(3,:)); 
cam.plot(P);

xy = cam.project(P);
 
R = cam.T.R;
R = R';
K = cam.K
%    
flen = K(1,1);
u0   = K(1,3);
v0   = K(2,3);
r00  = R(1,1);
r01  = R(1,2);
r02  = R(1,3);
r10  = R(2,1);
r11  = R(2,2);
r12  = R(2,3);
r20  = R(3,1);
r21  = R(3,2);
r22  = R(3,3);

a = flen*r00+u0*r20;
b = flen*r01+u0*r21;
c = flen*r02+u0*r22;
d = (-flen*r02-u0*r22);
e = flen*r10+v0*r20;
f = flen*r11+v0*r21;
g = flen*r12+v0*r22;
h = (-flen*r12-v0*r22);
i = r20;
j = r21;
k = r22;
l = -r22;

% image plane coordinates for the head
u_g = xy(1,1);
v_g = xy(2,1);
u_t = xy(1,2);
v_t = xy(2,2);
Z_t = 1.8;

%Ax = b
A = [ (d-u_g*l) (a-u_g*i) (b-u_g*j)    0         0;
      (h-v_g*l) (e-v_g*i) (f-v_g*j)    0         0;
      (d-u_t*l)     0         0     (a-u_t*i) (b-u_t*j);
      (h-v_t*l)     0         0     (e-v_t*i) (f-v_t*j)];
  
B = [     0       ;
          0       ;
     Z_t*(u_t*k-c);
     Z_t*(v_t*k-g)];

A\B
mldivide(A,B)
 