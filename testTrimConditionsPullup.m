% testTrimConditionsPullup.m
% Script to calculate steady pull up /pull over flight conditions for a fixed wing aircraft

% recall aircraft data structure
Navion_aircraft

% set CG positions relative to nominal CG;
deltaCGb  = [0;0;0];

%Flight conditions
V = aircraft.V;
h = aircraft.h;
pitchRate = 5 * pi/180;

%Calculate stady pull up /pull over flight conditionv
[alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb,aircraft)

%calculate trush requiered
Treq = aircraft.Tmax*deltat
Treq_lb = Treq/4.448222

% calculates angles in degrees

alphadeg = alpha*180/pi
ihdeg = ih*180/pi
deltaedeg = deltae*180/pi





