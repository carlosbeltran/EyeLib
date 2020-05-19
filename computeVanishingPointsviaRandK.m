function [v1,v2,v3] = computeVanishingPointsviaRandK(R,K)
% Get vanishing point provided rotation matrix cRw and intrinsics K 

v1 = K * R(:,1);
v1 = v1/v1(3)
v2 = K * R(:,2);
v2 = v2/v2(3)
v3 = K * R(:,3);
v3 = v3/v3(3)

end

