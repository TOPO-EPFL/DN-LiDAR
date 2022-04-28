function R = R1(a) 
% returns 3x3 rotation matrix around x-axis
% in right-hand frame sense (ref->new) as:
%          {  1     0     0   }
%  R1(a) = {  0    cosa  sina }
%          {  0   -sina  cosa }
%
% input: angle in radians!

R = zeros( 3,3) ;
R(1,1) =  1.0 ;
R(2,2) =  cos(a);
R(2,3) =  sin(a);
R(3,2) = -sin(a);
R(3,3) =  cos(a);
