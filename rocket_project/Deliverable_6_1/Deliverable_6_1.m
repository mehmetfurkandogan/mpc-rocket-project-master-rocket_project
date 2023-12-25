addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
x = zeros(12, 1);

H = 10; % Horizon length in seconds 
nmpc=NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15deg 
ref = @(t, x) refTVC(t); 

% MPC reference with specified maximum roll = 50deg 
% rollmax = deg2rad(50); 
% ref = @(t, x) refTVC(t, rollmax); 

% Evaluate once and plot optimal open−loop trajectory, 
% pad last input to get consistent size with time and state 
[u, Topt, Xopt, Uopt] = nmpc.get_u(x, ref); 
Uopt(:,end+1) = nan; 
ph=rocket.plotvis(Topt, Xopt, Uopt, ref); 

Tf = 30; 
[T,X,U,Ref] = rocket.simulate(x0,Tf, @nmpc.get_u, ref);