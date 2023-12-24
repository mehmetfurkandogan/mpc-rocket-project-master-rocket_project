addpath(fullfile('..', 'src'));

close; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
%% environment variables
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 20*Ts; % Closed-Loop Horizon length in seconds
Tf = 10;

%% mpc x
x0_x = [0 0 0 0]';
x_ref = [0 0 0 -4];

% Open Loop Trajectory
mpc_x = MpcControl_x(sys_x, Ts, Tf);
U_opt = mpc_x.get_u(x0_x, x_ref);
X_opt = X_opt + xs(1:4);
U_opt = U_opt + us(1);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

% Closed Loop Trajectory
mpc_x = MpcControl_x(sys_x, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0_x, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
%% mpc y
x0_y = [0 0 0 0]';
x_ref = [0 0 0 -4];

% Open Loop Trajectory
mpc_y = MpcControl_y(sys_y, Ts, Tf);
[~, T_opt, X_opt, U_opt] = mpc_y.get_u(x0_y, x_ref);
X_opt = X_opt + xs(5:8);
U_opt = U_opt + us(2);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

% Closed Loop Trajectory
mpc_y = MpcControl_y(sys_y, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0_y, Tf, @mpc_y.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
%% mpc z 
x0_z = [0 0]';
x_ref = [0 -4];

% Open Loop Trajectory
mpc_z = MpcControl_z(sys_z, Ts, Tf);
[~, T_opt, X_opt, U_opt] = mpc_z.get_u(x0_z, x_ref);
X_opt = X_opt + xs(9:10);
U_opt = U_opt + us(3);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

% Closed Loop Trajectory
mpc_z = MpcControl_z(sys_z, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0_z, Tf, @mpc_z.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
%% mpc roll
x0_roll = [0 0]';
x_ref = [0 deg2rad(35)];

% Open Loop Trajectory
mpc_roll = MpcControl_roll(sys_roll, Ts, Tf);
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0_roll, x_ref);
X_opt = X_opt - xs(11:12);
U_opt = U_opt - us(4);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

% Closed Loop Trajectory
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0_roll, Tf, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);