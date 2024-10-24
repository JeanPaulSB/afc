% ftrimTurn.m
% function to be minimized to obtain steady turn flight conditions
% usage
%   y = ftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft)
% where
%   Xi = [phi,theta;alpha;deltat;pitchControl;deltaa;deltar] : vector with variables to be found
%     with
%       phi : roll (rad)
%       theta : pitch (rad)
%       alpha : angle of attack (rad)
%       deltat : propulsion system control (0 <= deltat <= 1)
%       pitchControl : pitchControl,
%                      pitchControl = deltae and ih = ihTrimRect if Cmdelte ~= 0
%                      pitchControl = ih and deltar = 0 otherwise
%       deltaa : aileron (rad)
%       deltar : rudder (rad)
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   turnRate : turn rate (rad/s)
%   ihTrimRect : horizontal tail incidence control in equivalent steady rectilinear flight
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   y : value of funtion to be minimized to obtain steady turn flight conditions
%

function y = ftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft)
% declare Vbdot as a global variable
  global Vbdot

% extract components of Xi
  phi = Xi(1,1);
  theta = Xi(2,1);
  alpha = Xi(3,1);
  deltat = Xi(4,1);
  pitchControl = Xi(5,1);
  if aircraft.Cmdeltae ~= 0
    ih = ihTrimRect;
    deltae = pitchControl;
  else
    ih = pitchControl;
    deltae = 0;
  end
  deltaa = Xi(6,1);
  deltar = Xi(7,1);

% define steady rectilinear flight conditions for xdot, x, delta
  t = 0;
  Vwe = [0;0;0];

% set conditions for x
  pe = [0;0;-h];
  psi = 0; % steady rectilinear flight condition
  Phi = [phi;theta;psi];

% given that Vwe = [0;0;0], Vb = Vrelb
  beta = 0; % steady rectilinear flight condition
  Vb = [V*cos(beta)*cos(alpha);V*sin(beta);V*sin(alpha)*cos(beta)];
  Phidot = [0;0;turnRate];
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

% assemble control vector
  delta = [deltat;deltaf;ih;deltae;deltaa;deltar];

  y = norm(xdot-faircraft(t,x,delta,Vwe,deltaCGb,aircraft))^2+(pedot(3)+Vvert^2);
end
