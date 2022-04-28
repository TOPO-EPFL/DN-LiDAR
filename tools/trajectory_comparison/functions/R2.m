function R = R2(a) 
% returns 3x3 rotation matrix around y-axis
% in right-hand frame sense (ref->new) as:
%          { cosa   0   -sina }
%  R2(a) = {  0     1     0   }
%          { sina   0    cosa }
%
% input: angle in radians!

R = zeros(3,3) ;
R(1,1) =  cos(a);
R(1,3) = -sin(a);
R(2,2) =  1.0;
R(3,1) =  sin(a);
R(3,3) =  cos(a);
