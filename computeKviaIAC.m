%% Function Name: computeKviaIAC
%
% Description: Computes the Image of the Absolute Conic (IAC) and the
% intrinsics matrix (K) from three orthogonal finite vanishing points.
% Implements algorithm 8.2 "Computing K from scene and internal
% constraints" in the case of three vanishing points with zero skew and 
% square pixels (Example 8.27) from the book Multiple View Geometry in 
% Computer Vision by Richard Hartley and Andrew Zisserman (Second Edition).
% Pag.225 and pag 226.
%
% Assumptions: 
%   - vanishing points are orthogonal and finite representing x y and z 
%   directions. No-noise is assumed.
%   - Constraints (needs 5 constraints to solve W):
%       - zero skew (first constraint) (w21 = w12 = 0)
%       - square pixels (second constraint) (w11 = w22)
%       - three other contraints come from combining pairs of orothogonal 
%       vanishing points by u'wv = 0
%   - IAC form:
%    W = [w1 0  w2;
%         0  w1 w3;
%         w2 w3 w4]
%   - K form:
%    K = [f 0  u0;
%         0 f  v0;
%         0 0  1]
%    f   = focal lenght in pixels
%    u0,v0 = image center
%
% Inputs:
%   The three vanishing points in homogeneous coordinates
% Outputs:
%   W Image of the Absolute Conic
%   K intrinsics matrix
%
% $Revision: v1.0$ 
% $Author: Carlos Beltran-Gonzalez$
% $Email: carlos.beltran@iit.it$
% Istituto Italiano di Tecnologia
% Pattern Analysis and Computer Vision
% $Date:  May 16, 2020$
% Copyright: 
% Creative Commons Attribution-NonCommercial 4.0 International 
% (CC BY-NC 4.0) plus
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
% THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR 
% OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
% ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
% OTHER DEALINGS IN THE SOFTWARE.
%
%---------------------------------------------------------

function [K,W] = computeKviaIAC(vp1,vp2,vp3)

% Get vectorization of pairs of vanishing points
a1 = getvpVectorization(vp1,vp2);
a2 = getvpVectorization(vp1,vp3);
a3 = getvpVectorization(vp3,vp2);

A = vertcat(a1',a2',a3'); % Build up matrix A
w = null(A); % the w vector is the null space of A
W = [ w(1) 0 w(2);0 w(1) w(3);w(2) w(3) w(4)]; % Build the IAC from w
K = inv(chol(W)); % Get intrinsic matrix (K) by Cholesky factorization of W 
K = K./K(9); % normalize K by forcing k33 to 1

end

function a = getvpVectorization(u,v)
% input two vanishing points u and v
% u'wv =
% [u1 u2 u3] [w1 0 w2;0 w1 w3;w2 w3 w4] [v1 v2 v3]' =
% [u1 u2 u3] [w1v1+w2v3; w1v2+w3v3;w2v1+w3v2+w4v3]  =
% u1(w1v1+w2v3) + u2(w1v2+w3v3) + u3(w2v1+w3v2+w4v3)=
% w1(v1u1+v2u2) + w2(v3u1+v1u3) + w3(v3u2+v2u3) + w4v3u3
% then considering a vectorization a'w = 0 we have:
% a = (v1u1+v2u2,v3u1+v1u3,v3u2+v2u3,v3u3)
% w = (w1,w2,w3,w4)

a = [v(1)*u(1)+v(2)*u(2),
     v(3)*u(1)+v(1)*u(3),
     v(3)*u(2)+v(2)*u(3),
     v(3)*u(3)];
end

