function R = R3(a) 
% returns 3x3 rotation matrix around z-axis
% in right-hand frame sense (ref->new) as:
%          { cosa  sina   0   }
%  R3(a) = {-sina  cosa   0   }
%          {  0     0     1   }
%
% input: angle in radians!

R = zeros(3,3) ;
R(1,1) =  cos(a);
R(1,2) =  sin(a);
R(2,1) = -sin(a);
R(2,2) =  cos(a);
R(3,3) =  1.0;
