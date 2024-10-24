% testTrimConditionsRoll.m
% Script to calculate steady rectilinear flight conditions for a fixed wing aircraft

% recall aircraft data structure
Navion_aircraft

% set CG positions relative to nominal CG;
deltaCGb  = [0;0;0];

%Flight conditions
V = aircraft.V;
h = aircraft.h;

rollRate = 10 * pi/180;


%calculate steady turn flight conditios for this aircraft

[theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb,aircraft)

%calculate trush requiered
Treq = aircraft.Tmax*deltat
Treq_lb = Treq/4.448222

% calculates angles in degrees

thetadeg = theta*180/pi
alphadeg  =alpha *180/pi
betadeg = beta*180/pi
ihdeg = ih*180/pi % modificar en turn
deltaedeg = deltae*180/pi
deltaadeg = deltaa*180/pi
deltardeg = deltar*180/pi


