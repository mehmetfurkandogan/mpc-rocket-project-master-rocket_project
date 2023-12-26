addpath(fullfile('..', 'src'));

close all
clear 
clc

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
%% 
% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x0 = zeros(12,1);

% Setup reference function (ref4)
ref4 = [2 2 2 deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = mpc.get_u(x0, ref4);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4); % Plot as usual
exportgraphics(ph.fig,'plots/ref4.eps', BackgroundColor='none',ContentType='vector')

% Setup reference function (TVC)
ref = @(t , x ) ref_TVC(t);
% Simulate
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
% Visualize
rocket.anim_rate = 10; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
exportgraphics(ph.fig,'plots/tvc.eps', BackgroundColor='none',ContentType='vector')