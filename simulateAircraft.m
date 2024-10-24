% simulateAircraft.m
% function to simulate a fixed wing aircraft modeled by faircraft function
% usage
%   function [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
% where
%   tfinal : final simulation time (s)
%   x0 = [pe0;Phi0;Vb0;Omegab0] : initial state vector
%   delta = [deltat;deltaf;ih;deltae;deltaa;deltar] : aircraft controls vector
%     with
%       deltat : propulsion system control (0<=deltat<=1)
%       deltaf : flaps defflection (rad)
%       ih : horizontal tail incidence (rad)
%       deltae : elevator defflection (rad)
%       deltaa : aileron defflection (rad)
%       deltar : rudder defflection (rad)
%   Vwe : wing velocity expressed in earth frame (m/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   t : times vector (s)
%   X : matrix such that row i es x' for time t(i)
%   Y : matrix such that row is [V;alpha;beta] for time t(i)
%

function [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
% initialize global variable vbdot
  global Vbdot
  Vbdot = [0;0;0];

% set numerical methods parameters
  options = odeset('AbsTol',1e-3,'RelTol',1e-3,'InitialStep',0.01,'MaxStep',0.01);

% solve ecuations of motion
  [t,X] = ode45(@(t,x) faircraft(t,x,delta,Vwe,deltaCGb,aircraft),[0 tfinal],x0,options); %[t,y] = ode45(odefun,tspan,y0,options)

% calculate airspeed, angle of attack, and angle of sideslip for each t(i)
  n = length(t);
  Y = zeros(n,3);
  for i=1:n
     Phi = X(i,4:6)';
     Vb = X(i,7:9)';
     Cbe = DCM(Phi);
     Vrelb = Vb-Cbe*Vwe;
     V = norm(Vrelb);
     alpha = atan(Vrelb(3)/Vrelb(1));
     beta = asin(Vrelb(2)/V);
     Y(i,:) = [V,alpha,beta];
  end
end
