% testfaircraft.m
% script to tesºººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººººt faircraft function
%

format long g

% recall Navion aircraft parameters structure
Navion_aircraft

% define values of arguments of faircraft
t = 0
pe = [0;0;-aircraft.h]
phi = 20*pi/180
theta = 30*pi/180
psi = 250*pi/180
Phi = [phi;theta;psi]
V = aircraft.V
alpha = 3*pi/180
beta = -5*pi/180
Vrelb = [V*cos(alpha)*cos(beta);V*sin(beta);V*sin(alpha)*cos(beta)]
Vwe = [10;-5;5]*1852/3600
Cbe = DCM(Phi)
Vb = Vrelb+Cbe*Vwe
p = 10*pi/180
q = 5*pi/180
r = -3*pi/180
Omegab = [p;q;r]
x = [pe;Phi;Vb;Omegab]
deltat = 0.5
deltaf = 0
ih = 0
deltae = 0.5*pi/180
deltaa = 0.1*pi/180
deltar = -0.5*pi/180
delta = [deltat;deltaf;ih;deltae;deltaa;deltar]
deltaCGb = [0.1;0;0]

% initialize Vbdot
global Vbdot
Vbdot = [-4;9;-4]

[xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb,aircraft)
