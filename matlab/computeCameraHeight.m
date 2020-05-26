function Z = computeCameraHeight(v1,v2,v3,b,t,Zt)
% Gets the camera height from the vanishing points and the height of a
% reference object with known botton and top image projections
% REFERENCE: Criminisi, Reid and Zisserman. Single View Metrology. 1999

hoz    = cross(v1,v2); % Get the horizon line

%% eventualy needed for method 1
%btLine = cross(b,t); % Get the line connecting b and t
%%Get the point interception between b->t line and horizon line
%i      = cross(btLine,hoz);
%i = i/i(3); % Normalize i
%%%%%%

hoznor = hoz/norm(hoz); % Normalize the horizon line

% Compute the scaling factor using the known target height
alpha = (-norm(cross(b,t)))/(dot(hoznor,b)*norm(cross(v3,t))*Zt)

%Method 1 needs i
%Z    = (-norm(cross(b,i)))/(dot(hoznor,b)*norm(cross(v3,i))*alpha)

% Compute the camara heigh using the previusly compute scaling factor
% Method 2 doesn't need i
Z   = -(inv(dot(hoznor,v3)))/alpha

end

