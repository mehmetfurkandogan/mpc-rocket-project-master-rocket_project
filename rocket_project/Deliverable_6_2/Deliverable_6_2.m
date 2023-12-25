addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/40; 
rocket = Rocket(Ts);
delay = 2;
H = 20*Ts;
nmpc = NmpcControl(rocket, H, delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 10;
rocket.mass = 1.75;
rocket.delay = delay; 
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);

