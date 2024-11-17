% Nonlinear project 2023-2024
% Group: Rositani
% Authors:
% Rositani Marco 10937256     
% Tosto Giuseppe 10934977
% Chiereghin Guglielmo 10666842      

clear
clc
close all

%% Data
% System data
Jl = 4e-4;
Bl = 0;
k = 0.8;
m = 0.3;
g = 9.8;
l = 0.3;
Jm = Jl;
Bm = 0.015;

%% Equilibrium values
u_bar=m*g*l*cos(pi/4);
x3_bar=m*g*l/k*cos(pi/4)+pi/4;
x1_bar=pi/4;
x_bar=[x1_bar,0,x3_bar,0]';
x0=[x1_bar,0,x3_bar,0]'; 

%% Matrices of the linearized system around the equilibrium xbar
A_lin = [   0                       1       0       0
            m*g*l/(Jl*sqrt(2))-k/Jl -Bl/Jl  k/Jl    0
            0                       0       0       1
            k/Jm                    0       -k/Jm   -Bm/Jm];

B_lin = [   0   0   0   1/Jm]';

C_lin = [   1   0   0   0];

D_lin = 0;

%% Extended system for pole placement with an integrator
Aext = [A_lin [0;0;0;0];-C_lin 0];
Bext = [B_lin;0];

%% Feedback linearization state space

M = [0 1 0 0;0 0 1 0; 0 0 0 1; 0 0 0 0]; 
N = [0;0;0;1]; 
Cfl = [1 0 0 0];
Dfl = 0;

% Extended system for pole placement with integrator on the feedback linearized system
Mext = [M [0;0;0;0];-Cfl 0];
Next = [N;0];

% Pole placement's gains
kppext = place(Aext,Bext,[-20 -25 -30 -35 -40]);
kppext_fl = place(Mext,Next,[-20 -25 -30 -35 -40]);


% Observer's gains
Kobs = place(A_lin',C_lin',[-200,-250,-300,-350])';
Kobsfl = place(M',Cfl',[-200,-250,-300,-350])';



%% Variable structure control

syms s
q=10;
r=100;
X=(s+20)*(s+25)*(s+30); % prescribed characteristc polynomial on the sliding surface
Beta=sym2poly(X); % beta coefficients to use in the variable structure control

%VSC extended to reduce the input switching frequency
M_VSCext=[M N;zeros(1,5)]; %extended matrices for the observer
Cfl_ext=[Cfl 0];
Kobsfl_VSCext = place(M_VSCext',Cfl_ext',[-200,-250,-300,-350,-400])';
X_integrator=(s+20)*(s+25)*(s+30)*(s+35);
Beta_integrator=sym2poly(X_integrator); % beta coefficients to use in the variable structure control with the integrator

%% Simulation results
step_amp=pi/4+pi/8;
step_time=1;



