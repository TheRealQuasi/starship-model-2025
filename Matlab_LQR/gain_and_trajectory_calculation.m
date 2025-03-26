close all 
clear
%clc


% Constants
J = 0.304;
Jx = 0.215;
Jy = 0.226;
h1 = 0.455933;
h2 = 0.330933;
g = 9.82;
m = 2.5;


% A, B, C and D matrices
   %xdot %gam  gdot ydot gam2 g2dot z  zdot
A = [0    g/2   0    0    0    0    0    0;   %xdot
     0    0    1    0    0    0    0    0;   %gam
     0    0    0    0    0    0    0    0;  %gdot
     0    0    0    0    g/2   0    0    0;  %ydot
     0    0    0    0    0    1    0    0;  %gam2
     0    0    0    0    0    0    0    0;  %g2dot
     0    0    0    0    0    0    0    1;  %z
     0    0    0    0    0    0    0    0]; %zdot

B =  [0          g       0;
      0          0       0;
      0          m*g*h1/(2*Jy)  0 ;
      0          0       g;
      0          0       0;     
      0          0       m*g*h2/(2*Jx);
      0          0       0;
      2/m        0       0];

C = eye(8,8);

D = zeros(8,3);
% Q and R matrices

Q = [6  0   0   0   0   0   0   0;  %xdot
     0   8   0   0   0   0   0   0; %gamma
     0   0   1   0   0   0   0   0; %gammadot
     0   0   0   6  0   0   0   0;  %ydot
     0   0   0   0   8   0   0   0; %beta
     0   0   0   0   0   1   0   0; %betadot
     0   0   0   0   0   0   8   0;  %z
     0   0   0   0   0   0   0   4];%zdot

R = [1  0   0; %thrust
     0    0.1   0;  %theta1
     0    0   0.1]; %theta2

% State space model
sys = ss(A,B,C,D);

sysd = c2d(sys,0.01);
Ad = sysd.A;
Bd = sysd.B;
% Calculate K
[K,S,P] = dlqr(Ad,Bd,Q,R);


% Trajectory planning
tspan = 0:0.01:50; 
% Calculate liftoff trajectory
z = 1; % Final altitude
r = 2; % "Gain" of the trajectory
z0 = 0.001;
[t,x0] = ode45(@(t,x) r*x*(1-x/z), tspan, z0);
% Add set points for trajectory
for i=1:1001
    zref(i,2) = x0(i);
    zref(i,1) = tspan(i);
end
for i=1:1001
    j = 1002-i;
    zref2(j,2) = x0(i);
    zref2(j,1) = tspan(i);
end
for i=1001:2000
    zref(i,2) = zref2(i-1000,2);
    zref(i,1) = tspan(i);
end
zref(2001,2) = 0;
zref(2001,1) = 30;

% Extract simulation data for the derivative reference
% derivative_ref = out.simout;
% derivative_time = out.tout;
% for i=1:length(derivative_time)
%     zdotref(i,1) = derivative_time(i);
%     zdotref(i,2) = derivative_ref(i);
%     zdotref(i,3) = zref(i,2);
% end

% Create csv file for the reference values to use for altitude and
% velocity
% writematrix(zdotref(1:1001,:),'zref.csv')

