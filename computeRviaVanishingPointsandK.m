function R = computeRviaVanishingPointsandK(K,vp1,vp2)
% Solves for cRw provided the vanishg points and the intrinsics matrix K
% vanishingpoints are expected as column vectors in normalized homogeneous 
% coordinates

r1 = (inv(K)*vp1)/norm((inv(K)*vp1));
r2 = (inv(K)*vp2)/norm((inv(K)*vp2));
r3 = cross(r1,r2);
R  = horzcat(r1,r2,r3);

end

