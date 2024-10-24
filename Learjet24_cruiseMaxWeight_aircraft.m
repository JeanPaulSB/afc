%
% Aircraft: Learjet 24
% Flight condition: Cruise (max wht)
% Altitude (ft): 40000
% Airspeed (kn): 401
% Mach number: 0.7
% Script name: Learjet24_aircraft.m
%
% Aircraft data source
%   Roskam, J. (2001), "Airplane Flight Dynamics and Automatic Flight Controls," Part I, DARcorporation.
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
% Editor:
% Juan Diego Gomez S.-Emilio Guerrero-Jean Paul Sierra
% En C_DU hace refereancia a V/V0, siendo la derivada de CD respecto a u, siendo derivada de CD respecto a V=V0
% El CD de roscam es un modelo diferente al que usamos en el curso, roskam es linealizado y el
% del curso es el del drag polar, un modelo parabolico, por lo que no se pude usar el CDo de roskam
% es importante determinar si la convencion de la literatura es congruente con el modelo de uso
% aircraft data
aircraft.aircraftName = 'Learjet 24';
aircraft.flightCondition = 'Cruise';
% aircraft flight contitions data;
aircraft.h = 40000*0.3048;
aircraft.V = 401*1852/3600;
aircraft.g = 9.80665;
% aircraft geometry
aircraft.S = 230*0.3048^2;
aircraft.b = 34*0.3048;
aircraft.cbar = 7*0.3048;
aircraft.A = aircraft.b^2/aircraft.S;
% aircraft aerodynamic parameters
aircraft.M0 = 0.7;
aircraft.CD0 = 0.025;
aircraft.CLmindrag = 0.0774;
aircraft.e = 0.82;
CDu = 0.104;
aircraft.CDM = CDu/aircraft.M0;
CYbeta = -0.73;
CYdeltaa = 0;
CYdeltar = 0.14;
CYp = 0;
CYr = 0.4;
aircraft.CCbeta = -CYbeta;
aircraft.CCdeltaa = -CYdeltaa;
aircraft.CCdeltar = -CYdeltar;
aircraft.CCp = -CYp;
aircraft.CCr = -CYr;
aircraft.CL0 = 0.13;
aircraft.CLalpha = 5.84;
aircraft.CLdeltaf = 0;
aircraft.CLih = 0.94;
aircraft.CLdeltae = 0.46;
aircraft.CLalphadot = 2.2;
aircraft.CLq = 4.7;
%aircraft.CLdeltaa = -0.152;
CLu = 0.28;
aircraft.CLM = CLu/aircraft.M0;
aircraft.Clbeta = -0.11;
aircraft.Cldeltaa = -0.178;
aircraft.Cldeltar = 0.178;
aircraft.Clp = -0.45;
aircraft.Clr = 0.16;
aircraft.Cm0 = 0.05;
aircraft.Cmalpha = -0.64;
aircraft.Cmdeltaf = 0;
aircraft.Cmih = -2.5;
aircraft.Cmdeltae = -1.24;
aircraft.Cmalphadot = -6.7;
aircraft.Cmq = -15.5;
Cmu = 0.05;
aircraft.CmM = Cmu/aircraft.M0;
aircraft.Cnbeta = 0.127;
aircraft.Cndeltaa = 0.02;
aircraft.Cndeltar = -0.074;
aircraft.Cnp = -0.008;
aircraft.Cnr = -0.2;
% aircraft mass and inertia parameters
aircraft.m = 13000*4.448222/aircraft.g;
aircraft.Ix = 28000*14.593903*0.3048^2;
aircraft.Iy = 18800*14.593903*0.3048^2;
aircraft.Iz = 47000*14.593903*0.3048^2;
aircraft.Ixz = 1300*14.593903*0.3048^2;
aircraft.Ib = [aircraft.Ix           0        -aircraft.Ixz;
                       0          aircraft.Iy         0;
                -aircraft.Ixz         0         aircraft.Iz];
aircraft.Ibinv = inv(aircraft.Ib);
% aircraft propulsion system parameters
aircraft.Tmax = 2950*2*4.448222;
