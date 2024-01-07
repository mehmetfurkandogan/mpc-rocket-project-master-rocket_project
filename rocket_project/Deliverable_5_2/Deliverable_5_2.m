addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
%% environment variables
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 50*Ts; % Closed-Loop Horizon length in seconds
%% mpc definitions
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
%% Simulation
Tf = 7;
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';
% Manipulate mass for simulation
rocket.mass = 2.13;
rocket.mass_rate = -0.27;
%% 
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
rocket.anim_rate = 3; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
exportgraphics(ph.fig,'plots/decreasing_mass.eps', BackgroundColor='none',ContentType='vector')