clear all;
clc;
syms d2;
syms d6;
q = sym('q%d', [6 1]);     % q1,q2,q3,q3,q5,q6
d0_6  = [cos(q(1))*sin(q(2))*q(3)-sin(q(1))*d2 + d6*(cos(q(1))*cos(q(2))*cos(q(4))*sin(q(5))+cos(q(1))*cos(q(5))*sin(q(2))-sin(q(1))*sin(q(4))*sin(q(5)));sin(q(1))*sin(q(2))*q(3)-cos(q(1))*d2 + d6*(cos(q(1))*sin(q(4))*sin(q(5))+cos(q(2))*cos(q(4))*sin(q(1))*sin(q(5))+sin(q(1))*sin(q(2))*cos(q(5)));cos(q(2))*q(3)+d6*(cos(q(2))*cos(q(5))-cos(q(4))*sin(q(2))*sin(q(5)))];
Jv = [diff(d0_6,q((1))),diff(d0_6,q((2)))  diff(d0_6,q((3))),diff(d0_6,q((4)))  diff(d0_6,q((5))),diff(d0_6,q((6)))];
z0 = [0;0;1];
z1 = [-sin(q(1));cos(q(1));0];
z_0_2 = [0;0;0];
z2 = [cos(q(1))*sin(q(2));sin(q(1))*sin(q(2));cos(q(2))];
z3 = [cos(q(1))*sin(q(2));sin(q(1))*sin(q(2));cos(q(2))];
z4 = [-cos(q(1))*cos(q(2))*sin(q(4))-sin(q(1))*cos(q(4));-sin(q(1))*cos(q(2))*sin(q(4))+cos(q(1))*cos(q(4));sin(q(2))*sin(q(4))];
z5 = [cos(q(1))*cos(q(2))*cos(q(4))*sin(q(5))+cos(q(1))*cos(q(5))*sin(q(2))-sin(q(1))*sin(q(4))*sin(q( 5));
    cos(q(1))*sin(q(4))*sin(q(5))+cos(q(2))*cos(q(4))*sin(q(1))*sin(q(5))+sin(q(1))*sin(q(2))*cos(q(5));
    cos(q(2))*cos(q(5))-cos(q(4))*sin(q(2))*sin(q(5))];
Jw = [z0,z1,z_0_2,z3,z4,z5];
J = [Jv;Jw];
% Let the velocity of end effector are v1, v2 ,v3 and w1, w2 and w3
v1 = 1;
v2 = 2;
v3 = 1;
w1 = 2;
w2 = 3;
w3 = 1;
v = [v1;v2;v3;w1;w2;w3];
% Hence the joint velocities will be 
q = inv(J)*v