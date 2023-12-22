clc;
clear;

Ts = 1/20;

rocket= Rocket(Ts);

[xs,us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us) 
sys = rocket.linearize(xs,us); % Linearize the non linear model about trim point