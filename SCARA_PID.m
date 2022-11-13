clc
clear all

%Inverse kinematics
function [theata d3]=Inv_SCARA(O,R,a1,a2,d4)
 
alpha=atan2(R(1,2),R(1,1));
 
C2=((O(1,1)^2)+(O(2,1)^2)-(a1^2)-(a2^2))/(2*a1*a2);
 
S2=sqrt(1-(C2^2));
 
theata2=atan2(S2,C2);
 
theata1=(atan2(O(2,1),O(1,1)))-atan2((a2*sin(theata2)),(a1+a2*cos(theata2)));
 
theata4=theata1+theata2-alpha;
 
theata1=(theata1*180/pi);
theata2=(theata2*180/pi);
theata4=(theata4*180/pi);
 
theata=[theata1,theata2,theata4];
 
d3=d1-O(3,1)-d4;

%Jacobian
function J=Jacobian(th1,th2)
 
 a1=1;
 a2=1;
J=[(-a1*sind(th1)-a2*sind(th1+th2)),-a2*sind(th1+th2),0,0;
    (a1*cosd(th1)+a2*cosd(th1+th2)),a2*cosd(th1+th2),0,0;
    0,0,-1,0;0,0,0,0;0,0,0,0;1,1,0,-1];

%Trajectory tracking
function Path_PLaning(OI,OF,TH4) % function take as input initial and                  
                                 % final position and orientation

 
a1=1;
a2=1;
d1=1;                             % definition of robot parameters
d2=.1;
d=.1;
L=.3;

Xi=OI(1,1);
Yi=OI(2,1);                       % definition of initial position
Zi=OI(3,1);

flag=0;                            % make signal flag to 0 value

Xf=OF(1,1);
Yf=OF(2,1);                        % definition of final position    
Zf=OF(3,1);
 
if sqrt(((Xi)^2)+((Yi)^2))>2     % test of initial position in or out     
                                 % work space  
fprintf('Out of work space')
 
flag=1;                          % if yes make signal flag=1
end
 
if sqrt(Xf^2+Yf^2)>2             % test of final position in or out     
                                 % work space 
fprintf('Out of work space')
flag=1;                          % if yes make signal flag=1
end
 
if flag==0                    % if flag=0 in this case we are in work
                              % space

[T1 T2 T4 d3]=Scara_Inverse(Xi,Yi,Zi,d1,d2,a1,a2) %call function
                                                  %Scara_Inverse
                                             % returning T1 T2 T4 d3
                                             % for initial point
 
 J=Jacobian(T1,T2)                 %call function jacobian to test
                                   % singularity of initial point
  J11=[J(1:3,1:3)]
 S=det(J11)
 if S==0
     flag=1
    fprintf('Singularity')
 end
 
 
 
[T1f T2f T4 d3]=Scara_Inverse(Xf,Yf,Zf,d1,d2,a1,a2) %call function
                                                    %Scara_Inverse
                                              % returning T1 T2 T4 d3
                                              % for initial point 
 
J=Jacobian(T1f,T2f)                %call function jacobian to test
                                   % singularity of final point
 J11=[J(1:3,1:3)]
  S=det(J11)
 if S==0
     flag=1
    fprintf('Singularity')
 end
end
 
% 
if flag==0
    
     
slope = (Yf-Yi)/(Xf-Xi);           % we make calculation of slope of 
                                   % second portion of path
  inc=.01           
  
for Z=0:inc:Zi                  % begin of first portion of path, 
                                % make the value of inc=0.01
                                % in this portion we maintain the
                                % value of X and Y and the only 
                                % variation is in Z axis

    d3=d1-Z-d2;                        
  
 SCARA_plot(T1,T2,T4,a1,a2,d1,d3,d2,L,d)
 

hold on 
 plot3([Xi Xi],[Yi  Yi],[0 Zi],'red','linewidth',2) %plotting first 
                                                   % portion of path 

hold on
 
    
 
 plot3([Xi Xf],[Yi  Yf],[Zi Zi],'red','linewidth',2) %plotting second 
                                                   % portion of path 

hold on
 
plot3([Xf Xf],[Yf  Yf],[Zi Zf],'red','linewidth',2) %plotting third 
                                                   % portion of path 

hold off
 
 pause(.01)
end
if Xi>Xf                      % if the desired position is < the
                              % initial position we decrement xi by
                              % -0.01
    inc=-.01;
else
    inc=.01;                   % if the desired position is > the
                               % initial position we increment xi by
                               % +0.01
end
 
for X=Xi:inc:Xf           % for any new value of X and Y we calculate
                          % [T1 T2 T4 d2] by call of function 
                          %  Scara_Inverse 

    Y=slope*(X-Xf)+Yf;
 [T1 T2 T4 d3]=Scara_Inverse(X,Y,Zi,d1,d2,a1,a2);
 
plot3([Xi Xi],[Yi  Yi],[0 Zi],'red','linewidth',2)  %plotting first 
                                                   % portion of path 
hold on
plot3([Xi Xf],[Yi  Yf],[Zi Zi],'red','linewidth',2)%plotting second 
                                                    % portion of path 
hold on
plot3([Xf Xf],[Yf  Yf],[Zi Zf],'red','linewidth',2) %plotting third 
                                                    % portion of path 
hold on  
  
 SCARA_plot(T1,T2,T4,a1,a2,d1,d3,d2,L,d)     % ploting the motion of
                                             % robot on the path

 pause(.01)
 
end
    inc=-.01;                   % begin of third portion of path, 
                                % make the value of inc=-0.01
                                % in this portion we maintain the
                                % value of X and Y and the only 
                                % variation is in Z axis

for Z=Zi:inc:Zf
    d3=d1-Z-d2;                 % we go down by decrementing Zi

    
 SCARA_plot(T1,T2,T4,a1,a2,d1,d3,d2,L,d)     % ploting the motion of
                                             % robot on the path

 
 hold on 
 plot3([Xi Xi],[Yi  Yi],[0 Zi],'red','linewidth',2) %plotting first 
                                                    % portion of path 

hold on
 
 plot3([Xi Xf],[Yi  Yf],[Zi Zi],'red','linewidth',2) %plotting second 
                                                     % portion of path 

hold on
 
plot3([Xf Xf],[Yf  Yf],[Zi Zf],'red','linewidth',2)  %plotting third 
                                                    % portion of path

hold off
pause(.01)
end
 
 for T4=0:1:TH4            % adjustement and orientation of end 
                           % effector
hold on
 plot3([Xi Xi],[Yi  Yi],[0 Zi],'red','linewidth',2) %plotting first 
                                                    % portion of path 

plot3([Xi Xf],[Yi  Yf],[Zi Zi],'red','linewidth',2) %plotting second 
                                                    % portion of path 

 
 plot3([Xf Xf],[Yf  Yf],[Zi Zf],'red','linewidth',2) %plotting third 
                                                    % portion of path 

 

hold off
  SCARA_plot(T1,T2,T4,a1,a2,d1,d3,d2,L,d)     % ploting the motion of
                                             % robot on the path
hold on
plot3([Xi Xi],[Yi  Yi],[0 Zi],'red','linewidth',2) %plotting first 
                                                    % portion of path
 
plot3([Xi Xf],[Yi  Yf],[Zi Zi],'red','linewidth',2) %plotting second 
                                                    % portion of path

plot3([Xf Xf],[Yf  Yf],[Zi Zf],'red','linewidth',2) %plotting third 
                                                    % portion of path
 
  pause(.01) 
 
  end 
hold off
end


%Plot
function plotSCARA=SCARA_plot(T1,T2,T4,a1,a2,d1,d2,d3,L,d)

if T1<0;
    T1=T1+360;lcm
end
if T2<0;
    T2=T2+360;
end
if T4<0;
    T4=T4+360;
end
if T4>180
    T4=T4-180;
end
%%%%%%%%for a1
X_1=a1*cosd(T1);
Y_1=a1*sind(T1);
plot3([0 X_1],[0 Y_1],[d1 d1],'b','linewidth',7)

%%%%%%%%for a2
X_2=X_1+(a2*cosd(T1+T2));
Y_2=Y_1+(a2*sind(T1+T2));
hold on
plot3([X_2 X_1],[Y_2 Y_1],[d1 d1],'r','linewidth',7)

%%%%%%%%%%%%%%%%%%% for d1
hold on
plot3([0 0],[0 0],[0 d1],'black','linewidth',8)
%%%%%%%%%%%%%%%%%%%%% for d2
hold on
plot3([X_2 X_2],[Y_2 Y_2],[d1 d1-d2],'black','linewidth',5)
% plot3([X_2 X_2],[Y_2 Y_2],[d1 d1+.4],'blue','linewidth',7)    % tube
%%%%%%%%%%%% End effector
hold on
if (T4 < 90)
XP=[min(X_2-(L/2)*cosd(T4),X_2+(L/2)*cosd(T4)),max(X_2-(L/2)*cosd(T4),X_2+(L/2)*cosd(T4))];
YP=[min(Y_2-(L/2)*sind(T4),Y_2+(L/2)*sind(T4)),max(Y_2-(L/2)*sind(T4),Y_2+(L/2)*sind(T4))];
plot3([XP(1),XP(2)],[YP(1),YP(2)],[d1-d2 d1-d2],'linewidth',5)
hold on

XP=[min(X_2-(d/2)*cosd(T4),X_2+(d/2)*cosd(T4)),max(X_2-(d/2)*cosd(T4),X_2+(d/2)*cosd(T4))];
YP=[min(Y_2-(d/2)*sind(T4),Y_2+(d/2)*sind(T4)),max(Y_2-(d/2)*sind(T4),Y_2+(d/2)*sind(T4))];
plot3([XP(1),XP(1)],[YP(1),YP(1)],[d1-d2 d1-d2-d3])
hold on
plot3([XP(2),XP(2)],[YP(2),YP(2)],[d1-d2 d1-d2-d3])
else
XP=[min(X_2-(L/2)*cosd(T4),X_2+(L/2)*cosd(T4)),max(X_2-(L/2)*cosd(T4),X_2+(L/2)*cosd(T4))];
YP=[max(Y_2-(L/2)*sind(T4),Y_2+(L/2)*sind(T4)),min(Y_2-(L/2)*sind(T4),Y_2+(L/2)*sind(T4))];
plot3([XP(1),XP(2)],[YP(1),YP(2)],[d1-d2 d1-d2],'linewidth',5)
hold on

XP=[min(X_2-(d/2)*cosd(T4),X_2+(d/2)*cosd(T4)),max(X_2-(d/2)*cosd(T4),X_2+(d/2)*cosd(T4))];
YP=[max(Y_2-(d/2)*sind(T4),Y_2+(d/2)*sind(T4)),min(Y_2-(d/2)*sind(T4),Y_2+(d/2)*sind(T4))];
plot3([XP(1),XP(1)],[YP(1),YP(1)],[d1-d2 d1-d2-d3])
hold on
plot3([XP(2),XP(2)],[YP(2),YP(2)],[d1-d2 d1-d2-d3]) 
plot3([XP(2),XP(2)],[YP(2),YP(2)],[d1-d2 d1-d2-d3])    
hold on
end
% %%%%%%%%%%%%%%%%%%%% Joints
% hold on
% plot3([0 0],[0 0],[d1*.98 d1*1.02],'black','linewidth',5.5)
% figure
% plot([0 X_1],[0 Y_1])
% hold on
% plot([X_2 X_1],[Y_2 Y_1])
 axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2) 0 d1])
hold off