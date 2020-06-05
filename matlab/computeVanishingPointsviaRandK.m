%% Function Name: computeVanishingPointsviaRandK
% 
% Get vanishing point provided the rotation matrix cRw and intrinsics K 
%
% $Revision: v1.0$ 
% $Author: Carlos Beltran-Gonzalez$
% $Email: carlos.beltran@iit.it$
% Istituto Italiano di Tecnologia
% Pattern Analysis and Computer Vision
% $Date:  May 16, 2020$
% Copyright (C) 2020 Carlos Beltran-Gonzalez
% CopyPolicy: GNU Lesser General Public License v3

function [v1,v2,v3] = computeVanishingPointsviaRandK(R,K)

v1 = K * R(:,1);
v1 = v1/v1(3);
v2 = K * R(:,2);
v2 = v2/v2(3);
v3 = K * R(:,3);
v3 = v3/v3(3);

end

