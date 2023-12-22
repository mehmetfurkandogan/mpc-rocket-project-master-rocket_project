clc;
clear;

Ts = 1/20;
rocket = Rocket(Ts);

weight = rocket.mass*rocket.g;

Tf = 2.0; % Simulationendtime 
x0=[deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initialstate 
u =[deg2rad([0 0]), 62.667, 0]'; % (d1 d2 Pavg Pdiff) Constant input 
[T, X, U] = rocket.simulate(x0,Tf, u); % Simulate unknown, nonlinear model 
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time 
rocket.vis(T, X, U);