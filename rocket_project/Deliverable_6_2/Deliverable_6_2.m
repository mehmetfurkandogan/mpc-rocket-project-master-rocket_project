addpath(fullfile('..', 'src'));

close all; clear; clc;

%% TODO: This file should produce all the plots for the deliverable
% Compansated
Ts = 1/40; 
rocket = Rocket(Ts);
expected_delay = 5;
delay = 5;
H = 20*Ts;
nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 10;
rocket.mass = 1.75;
rocket.delay = delay; 
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
exportgraphics(ph.fig,'plots/delay_compansated_nmpc_corrected.eps', BackgroundColor='none',ContentType='vector')

%% Partially Compansated
Ts = 1/40; 
rocket = Rocket(Ts);
expected_delay = 4;
delay = 5;
H = 20*Ts;
nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 10;
rocket.mass = 1.75;
rocket.delay = delay; 
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
exportgraphics(ph.fig,'plots/delay_compansated_nmpc_partial.eps', BackgroundColor='none',ContentType='vector')

