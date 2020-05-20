clear all; close all;

Zc_gt = 5.2;
Zt    = 1.8;

% Plot word coordinate frame
Tcam = transl(0,0,Zc_gt)*trotx(deg2rad(-90))*troty(deg2rad(45))*trotx(deg2rad(-35));

cam = CentralCamera('focal', 0.015,'pixel',10e-6, ...
     'resolution',[1280 1024],'centre',[640 512],'pose',Tcam);

world = SE3();
trplot(world,'frame','0','color','b');

xlim([-1 5]);
ylim([-1 5]);
zlim([-0.1 5]);

% plot camera
cam.plot_camera();

% Plot points
hold on;
%%P = mkgrid(2,0.2,'pose',transl(0.7,0.7,0));
%P = mkcube(1.8,'pose',transl([4,4,0.9]));
%P = P(:,[1,5]);
P = [ 4 4; 4 4; 0 Zt];
plot3(P(1,:),P(2,:),P(3,:),'LineWidth',7); 
cam.plot(P);

B = P(:,1);
T = P(:,2);

xy = cam.project(P);

b = [xy(:,1);1];
t = [xy(:,2);1];
 
R = cam.T.R;
R = R';
K = cam.K
[v1,v2,v3] = computeVanishingPointsviaRandK(R,K);

hoz    = cross(v1,v2);
btLine = cross(b,t);
i      = cross(btLine,hoz);
i = i/i(3);

hoznor = hoz/norm(hoz);

alpha = (-norm(cross(b,t)))/(dot(hoznor,b)*norm(cross(v3,t))*Zt)
Zc    = (-norm(cross(b,i)))/(dot(hoznor,b)*norm(cross(v3,i))*alpha)
Zc2   = -(inv(dot(hoznor,v3)))/alpha