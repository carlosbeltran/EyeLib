%% Function Name: computeRviaVanishingPointsandK
% 
% Solves for cRw provided the vanishg points and the intrinsics matrix K
% vanishingpoints are expected as column vectors in normalized homogeneous 
% coordinates
%
% $Revision: v1.0$ 
% $Author: Carlos Beltran-Gonzalez$
% $Email: carlos.beltran@iit.it$
% Istituto Italiano di Tecnologia
% Pattern Analysis and Computer Vision
% $Date:  May 16, 2020$

function R = computeRviaVanishingPointsandK(K,vp1,vp2)

r1 = (inv(K)*vp1)/norm((inv(K)*vp1));
r2 = (inv(K)*vp2)/norm((inv(K)*vp2));
r3 = cross(r1,r2);
R  = horzcat(r1,r2,r3);

end

