% testSimulateAircraft.m
close all

% recall aircraft data structure
Navion_aircraft

% set CG relative position respect to nominal CG
deltaCGb = [0;0;0];

% set wind velocity
Vwe = [0;0;0];

% define simulation time
tfinal = 10;

% define initial conditions
h = aircraft.h;
V0 = (135-1)*1852/3600; %135 kn / small perturbation causes a decrease of 1 knot
alpha0 = 0.5012*pi/180; %0.5012 deg
beta0 = 0*pi/180; %0 deg
phi0 = 0*pi/180; %0 deg
theta0 = 0.5012*pi/180; %0.5012 deg
psi0 = 0*pi/180; %0 deg
deltat = 0.3369;
ih = 0*pi/180; %0 deg
deltae = -0.2524*pi/180; %-0.2524 deg
deltaa = 0*pi/180; %0 deg
deltar = 0*pi/180; %0 deg

% assemble x0 and delta
pe0 = [0;0;-h];
Phi0 = [phi0;theta0;psi0];
Vrelb0 = [V0*cos(alpha0)*cos(beta0);V0*sin(beta0);V0*sin(alpha0)*cos(beta0)];
Cbe = DCM(Phi0);
Vb0 = Vrelb0+Cbe*Vwe;
Omegab0 = [0;0;0];
x0 = [pe0;Phi0;Vb0;Omegab0];
deltaf = 0;
delta = [deltat;deltaf;ih;deltae;deltaa;deltar];

% simulate aircraft
[t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft);

% plot simulation results
plotAircraftSimulationResults(t,X,Y,aircraft);
