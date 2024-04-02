close all 
clear
%clc

J = 0.304;
h1 = 0.3;
h2 = 0.2;
g = 9.82;
Tm = 4;
m = 2.43;

F = 4;

%system dynamics
     %x  xdot gam  gdot y  ydot beta bdot  z    zdot
a = [0   1    0    0    0    0    0    0    0    0;
     0   0    -g   0    0    0    0    0    0    0;
     0   0    0    1    0    0    0    0    0    0;
     0   0    0    0    0    0    0    0    0    0;
     0   0    0    0    0    1    0    0    0    0;
     0   0    0    0    0    0    -g   0    0    0;
     0   0    0    0    0    0    0    1    0    0;
     0   0    0    0    0    0    0    0    0    0;
     0   0    0    0    0    0    0    0    0    1;
     0   0    0    0    0    0    0    0    1    0];

   %xdot %gam  gdot ydot bet bdot  z   zdot
A = [0    -g   0    0    0    0    0    0;
     0    0    1    0    0    0    0    0;
     0    0    0    0    0    0    0    0;
     0    0    0    0    -g   0    0    0;
     0    0    0    0    0    1    0    0;
     0    0    0    0    0    0    0    0;
     0    0    0    0    0    0    0    1;
     0    0    0    0    0    0    1    0];

B = [0          g       0;
     0          0       0;
     0          m*g*h1/J 0;
     0          0       g;
     0          0       0;
     0          0       m*g*h2/J;
     0          0       0;
     Tm/m       0       0];

    %x  xdot gam gdot y ydot be bdot z  zdot
C = eye(8);
 

% eye(10);    %[0  0   1   0   0   0   1   0   0   1];
   
D = zeros(8,3);    %0;  %[0  0   0 ];

%initial value and reference vector

%initial conditions:
x0 = [-10; %x
      0; %xdot
      0; %gamma
      0; %gammadot
      10; %y
      0; %ydot
      0; %beta
      0; %betadot
      8; %z
      0];%zdot

% %reference
% xd = [2; %x
%       0; %xdot
%       0; %gamma
%       0; %gammadot
%       2; %y
%       0; %ydot
%       0; %beta
%       0; %betadot
%       5; %z
%       0];%zdot
% %calculate ud
% ud = -inv(B'*B)*B'*A*xd;

% control law
q = [1  0   0   0   0   0   0   0   0   0; %x
     0  10   0   0   0   0   0   0   0   0;  %xdot
     0  0   5   0   0   0   0   0   0   0; %gamma
     0  0   0   1   0   0   0   0   0   0; %gammadot
     0  0   0   0   1   0   0   0   0   0; %y
     0  0   0   0   0   10   0   0   0   0;  %ydot
     0  0   0   0   0   0   5   0   0   0; %beta
     0  0   0   0   0   0   0   1   0   0; %betadot
     0  0   0   0   0   0   0   0   5   0;  %z
     0  0   0   0   0   0   0   0   0   1];%zdot

Q = [10   0   0   0   0   0   0   0;  %xdot
     0   5   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   10   0   0   0   0;  %ydot
     0   0   0   0   5   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   5   0;  %z
     0   0   0   0   0   0   0   1];%zdot
%Q = eye(10);
     
R = [1  0   0; %thrust
    0   1  0;  %theta1
    0   0   1]; %theta2


% Define the time span for simulation
tspan = 0:0.01:10; 

% Calculate K
sys = ss(A, B, C, D);
[K,S,P] = lqr(sys,Q,R);

sys2 = ss(A,B,C,D);
sysd = c2d(sys2,0.01);
[Kd,S2,P2] = lqr(sys2,Q,R);


%Qi = eye(16);

%[Ki,Si,e] = lqi(sysd,Q,R);

%rank(ctrb(A,B))
%rank(obsv(A,C))

Ae = [A   zeros(8,8);
      -C  zeros(8,8)];

Be = [B;
      zeros(8,3)];

Br = [0;
     1];

Qi = eye(16);

%sysde = ss(Ae,Be,C,D);
%sysde = c2d(sysde,0.01);
[Ke,Se,Pe] = lqi(sysd,Qi,R);


%%% KALMAN

Qk = [1   0   0   0   0   0   0   0;  %xdot
     0   1   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   1   0   0   0   0;  %ydot
     0   0   0   0   1   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   1   0;  %z
     0   0   0   0   0   0   0   1];%zdot

Rk = [1   0   0   0   0   0   0   0;  %xdot
     0   1   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   1   0   0   0   0;  %ydot
     0   0   0   0   1   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   1   0;  %z
     0   0   0   0   0   0   0   1];%zdot


%[kalmf,L,~,Mx,Z] = kalman(sysd,Qk,Rk);