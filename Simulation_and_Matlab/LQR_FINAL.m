close all 
clear
clc

% Constants
J = 0.304;
I_x = 0.215;
I_y = 0.226;
h_1 = 0.455933;
h_2 = 0.330933;
g = 9.82;
m = 2.5;

% Systemmatris utan fi
A = [0   1   0   0   0   0    0    0    0    0;
     0   0   0   0   0   0   g/2   0    0    0;
     0   0   0   1   0   0    0    0    0    0;
     0   0   0   0   0   0    0    0   g/2   0;
     0   0   0   0   0   1    0    0    0    0;
     0   0   0   0   0   0    0    0    0    0;
     0   0   0   0   0   0    0    1    0    0;
     0   0   0   0   0   0    0    0    0    0;
     0   0   0   0   0   0    0    0    0    1;
     0   0   0   0   0   0    0    0    0    0];
% Insignalmatris utan fi
B = [0          0             0            0;
     0          0             0            g/2;
     0          0             0            0;
     0          0             g/2          0;
     0          0             0            0;
     1/m       1/m            0            0;
     0          0             0            0;
     0          0      g*m*h_1/(2*I_y)     0;
     0          0             0            0;
     0          0             0     g*m*h_2/(2*I_x)];
%Utsignalmatris utan fi
C = eye(10);
% Utsignal/Insignalmatris utan fi
D = zeros(10,4);
% Skapa en tillstånds modell via State Space (ss)
system = ss(A, B, C, D);

% Define Q matrix
Q = diag([ ...
    0.01,  ... % x
    1,     ... % xdot
    0.01,  ... % y
    1,     ... % ydot
    100,   ... % z
    1,     ... % zdot
    10,    ... % gamma1
    1,     ... % gammadot1
    10,    ... % gamma2
    1      ... % gammadot2
]);

% Define R matrix
R = diag([ ...
    1,   ... % u1
    1,   ... % u2
    10,  ... % u3
    10   ... % u4
]);

% Discretesizing matrices
sysd = c2d(system,0.01);
Ad = sysd.A;
Bd = sysd.B;

% Compute LQR gain K
[K,S,P] = dlqr(Ad,Bd,Q,R)

