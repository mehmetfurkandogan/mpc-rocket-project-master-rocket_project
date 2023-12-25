addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
x0 = zeros(12, 1);

H = 20*Ts; % Horizon length in seconds 
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15deg
% ref = [2,2,2,deg2rad(40)]';
% ref = @(t_, x_)ref_TVC(t_/2); 

% MPC reference with specified maximum roll = 50deg 
rollmax = deg2rad(50); 
ref = @(t_, x_) ref_TVC(t_/2, rollmax); 

% Evaluate once and plot optimal open−loop trajectory, 
% pad last input to get consistent size with time and state 
% [u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref); 
% U_opt(:,end+1) = nan; 
% ph = rocket.plotvis(T_opt, X_opt, U_opt, ref); 

Tf = 60; 
[T,X,U,Ref] = rocket.simulate(x0,Tf, @nmpc.get_u, ref);
rocket.anim_rate = 3;
ph = rocket.plotvis(T, X, U, Ref);