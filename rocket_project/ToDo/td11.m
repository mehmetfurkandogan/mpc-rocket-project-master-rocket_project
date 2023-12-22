clc;
clear;

addpath("../src/")

Ts=1/20; 
rocket= Rocket(Ts); 
u=[d1, d2, Pavg, Pdiff]'; %(Assign appropriately) 
[bF, bM] = rocket.getForceAndMomentFromThrust(u); 
x=[w, phi, v, p]'; %(Assign appropriately) 
xdot = rocket.f(x,u);