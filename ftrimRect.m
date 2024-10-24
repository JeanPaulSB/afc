% ftrimRect.m
% function to be minimized to obtain steady rectilinear flight conditions
% usage
%   y = ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft)
% where
%   Xi = [alpha;deltat;pitchControl] : vector with variables to be found
%     with
%       alpha : angle of attack (rad)
%       deltat : propulsion system control (0 <= deltat <= 1)
%       pitchControl : pitchControl,
%                      pitchControl = ih if Cmih ~= 0
%                      pitchControl = deltae otherwise
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   y : value of funtion to be minimized to obtain steady rectilinear flight conditions
%

function y = ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft)
% declare Vbdot as a global variable
  global Vbdot

% extract components of Xi
  alpha = Xi(1,1);
  deltat = Xi(2,1);
  pitchControl = Xi(3,1);
  if aircraft.Cmih ~= 0
    ih = pitchControl;
    deltae = 0;
  else
    ih = 0;
    deltae = pitchControl;
  end

% define steady rectilinear flight conditions for xdot, x, delta
  t = 0;
  Vwe = [0;0;0];

% calculate flight path angle
  gamma = asin(Vvert/V);

% set conditions for x
  pe = [0;0;-h];
  phi = 0; % steady rectilinear flight condition
  theta = alpha+gamma; % steady rectilinear flight condition
  psi = 0;
  Phi = [phi;theta;psi];

% given that Vwe = [0;0;0], Vb = Vrelb
  beta = 0; % steady rectilinear flight condition
  Vb = [V*cos(beta)*cos(alpha);V*sin(beta);V*sin(alpha)*cos(beta)];
  Phidot = [0;0;0];
  Omegab = Hinv(Phi)*Phidot;

% assemble state vector x
  x = [pe;Phi;Vb;Omegab];

% set conditions for xdot
  Cbe = DCM(Phi);
  pedot = Cbe'*Vb;
  Vbdot = [0;0;0];
  Omegabdot = [0;0;0];

% assemble derivative of state vector xdot
  xdot = [pedot;Phidot;Vbdot;Omegabdot];

% set aircraft control
  deltaf = 0;
  deltaa = 0;
  deltar = 0;

% assemble control vector
  delta = [deltat;deltaf;ih;deltae;deltaa;deltar];

  y = norm(xdot-faircraft(t,x,delta,Vwe,deltaCGb,aircraft))^2;
end
