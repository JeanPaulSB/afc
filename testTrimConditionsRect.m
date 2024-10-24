% testTrimConditionsRect.m
% script to calculate steady rectilinear flight conditions for fixed wing aircraft
format long g

% recall aircraft data structure
Navion_aircraft;

% ser relative CG position respect to nominal CG
deltaCGb = [0;0;0];

% set flight conditions
V = aircraft.V;
h = aircraft.h;
Vvert = 0*0.3048/60;

% calculate steady flight conditions
[theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb,aircraft)

thetadeg = theta*180/pi
gammadeg = gamma*180/pi
alphadeg = alpha*180/pi
ihdeg = ih*180/pi
deltaedeg = deltae*180/pi

% calculate thrust required
thrustRequired = aircraft.Tmax*deltat
thrustRequired_lb = thrustRequired/4.448222
