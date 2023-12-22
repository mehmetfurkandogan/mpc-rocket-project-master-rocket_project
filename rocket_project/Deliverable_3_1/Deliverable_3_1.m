addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
%% environment variables
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 200*Ts; % Horizon length in seconds
Tf = 10;


%% mpc x
mpc_x = MpcControl_x(sys_x, Ts, H);
x0_x = [0 0 0 3]';
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0_x, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
%% mpc y
mpc_y = MpcControl_y(sys_y, Ts, H);
x0_y = [0 0 0 3]';
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0_y, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
%% mpc z 
mpc_z = MpcControl_z(sys_z, Ts, H);
x0_z = [0 3]';
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0_z, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
%% mpc roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
x0_roll = [0 1]';
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0_roll, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);