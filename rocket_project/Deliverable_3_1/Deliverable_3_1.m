addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
% Design MPC controller
H = 200*Ts; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input (x is the index of the subsystem here)
x0 = [0 deg2rad(5) 0 0]';
u_x = mpc_x.get_u(x0);

%% 
Tf = 5;
x0 = [0 deg2rad(5) 0 0]';
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
