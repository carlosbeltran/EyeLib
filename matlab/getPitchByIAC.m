%% Function Name: getPitchByIAC
% 
% Gets the angle between the optical axis and the vertical vanishg point
% by means of the image of the absolute conic
% K is expected to be column wise
% verticalvp in image coordinates
% angle is returned in degrees
% Note: check page 210 of Hartley and Zisserman " Multiple View Geometry in
% computer vision" [second edition]
%
% $Revision: v1.0$ 
% $Author: Carlos Beltran-Gonzalez$
% $Email: carlos.beltran@iit.it$
% Istituto Italiano di Tecnologia
% Pattern Analysis and Computer Vision
% $Date:  May 16, 2020$

function pitchDeg = getPitchByIAC(K,verticalvp)

x1  = K(:,3); % Image centre (first image point in homogeneus form)
iac = inv(K*K'); %Image of the absolute conic
x2  = [verticalvp,1]'; % Vanishing point (second image point)
costheta = (x1'*iac*x2)/(sqrt(x1'*iac*x1)*sqrt(x2'*iac*x2));
pitch_iac = rad2deg(pi/2 - acos(costheta));
pitchDeg = pitch_iac;

end

