clear all;
clc;
syms a2;syms m1;syms m2;syms m3;syms g
syms a1
A = sym('q%d', [3 1]);     %  This contains the angles q1,q2,q3
Ia = sym('Ia%d%d',[3,3]);   % moment of inertia for 1st link
Ib = sym('Ib%d%d',[3,3]);    % moment of inertia for 2nd link
Ic = sym('Ic%d%d',[3,3]);    % moment of inertia for 3rd link
Jvc1 = zeros(3); % Creating a zero matrix although this is the 1st row of jacobian and similarly all other rows 
Jvc2 =[-(a2/2)*cos(A(2))*sin(A(1)),-(a2/2)*cos(A(1))*sin(A(2)),0;
    (a2/2)*cos(A(2))*cos(A(1)),-(a2/2)*sin(A(1))*sin(A(2)),0;
    0,(a2/2)*cos(A(2)),0];
Jvc3 =[-(a2/2+A(3)/2)*cos(A(2))*sin(A(1)),-(a2/2+A(3)/2)*cos(A(1))*sin(A(2)),cos(A(2))*cos(A(1));
    (a2/2+A(3)/2)*cos(A(2))*cos(A(1)),-(a2/2+A(3)/2)*sin(A(1))*sin(A(2)),cos(A(2))*sin(A(1));
    0,(a2/2+A(3)/2)*cos(A(2)),sin(A(2))];
Jwc1 = [0,0,0;0,0,0;1,0,0];
Jwc2 = [0,-sin(A(1)),0;0,cos(A(1)),0;1,0,0];
Jwc3 = Jwc2;
C = [A(1),0,0;0,0,0;0,0,0];
D = m1*transpose(Jvc1)*Jvc1 + m2*transpose(Jvc2)*Jvc2 + m3*transpose(Jvc3)*Jvc3  + transpose(Jwc1)*Ia*Jwc1+ transpose(Jwc2)*Ib*Jwc2+ transpose(Jwc3)*Ic*Jwc3;
for k = 1:3
    for i=1:3
        for j= 1:3
            C(i,j,k) = (1/2)*(diff(D(k,j),A(i))+diff(D(k,i),A(j))-diff(D(i,j),A(k)));         
        end
    end
end

V_q = m1*g*a1/2 + (m2+m3)*g*(a1+(a2+A(3))*sin(A(2))); % Accounting for potential energy
phi = [A(1);0;0];
for k= 1:3
    phi(k) = diff(V_q,A(k));
end