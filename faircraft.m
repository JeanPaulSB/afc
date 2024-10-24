% faircraft.m
% function to define the mathematical model for a fixed wing aircraft
%   [xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb,aircraft)
% where
%   t : time (s)
%   x = [pe;Phi;Vb;Omegab] : state vector
%     with
%       pe = [xe;ye;ze] : aircraft CG position respect to earth expressed in earth frame (m)
%       Phi = [phi;theta;psi] : Euler angles (rad)
%       Vb = [u;v;w] : aircraft velocity respect to earth expressed in body frame (m/s)
%       Omegab = [p;q;r] : aircraft angular velocity respect to earth in body frame (rad/s)
%   delta = [deltat;deltaf;ih;deltae;deltaa;deltar] : aircraft controls vector
%     whit
%       deltat : propulsion system control (0<=deltat<=1)
%       deltaf : flaps defflection (rad)
%       ih : horizontal tail incidence (rad)
%       deltae : elevator defflection (rad)
%       deltaa : aileron defflection (rad)
%       deltar : rudder defflection (rad)
%       deltaaero = [deltaf;ih;deltae;deltaa;deltar] : aerodynamic controls vector (rad)
%   Vwe : wing velocity expressed in earth frame (m/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   xdot = [pedot;Phidot;Vbdot;Omegabdot] : derivative of state vector
%     with
%       pedot = Ve : aircraft CG velocity respect to earth expressed in earth frame (m/s)
%       Phidot = [phidot;thetadot;psidot] : derivative of Euler angles (rad/s)
%       Vbdot = [udot;vdot;wdot] : derivative of aircraft velocity respect to earth expressed in body frame (m/s^2)
%       Omegabdot = [pdot;qdot;rdot] : derivative of aircraft angular velocity respect to earth in body frame (rad/s^2)
%   y = [V;alpha;beta]
%     with
%       V : airspeed (m/s)
%       alpha : angle of attack (rad)
%       beta : angle of side slip (rad)
%

function [xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb,aircraft)
% declare Vbdot as a global variable
  global Vbdot

% extract components of x and delta
  pe = x(1:3,1); %from row 1 to 3, of column 1
  h = -pe(3);
  Phi = x(4:6,1);
  Vb = x(7:9,1);
  Omegab = x(10:12,1);
  deltat = delta(1,1);
  deltaaero = delta(2:6,1);

% evaluate translational kinematics
  Cbe = DCM(Phi); %DCM : direction cosine matrix function
  pedot = Cbe'*Vb;

% evaluate rotational kinematics
  Phidot = H(Phi)*Omegab;

% calculate weight
  Ge = [0;0;aircraft.g];
  Gb = Cbe*Ge;
  Wb = aircraft.m*Gb;

% calculate relative velocity
  Vrelb = Vb-Cbe*Vwe;

% calculate airspeed
  V = norm(Vrelb);

% calculate angle of attack
  alpha = atan(Vrelb(3)/Vrelb(1));

% calculate angle of side splip
  beta = asin(Vrelb(2)/V);

% calculate derivate of relative velocity
  Vrelbdot = Vbdot+cross(Omegab,Cbe*Vwe);

% calculate derivate of angle of attack
  alphadot = (Vrelbdot(3)*Vrelb(1)-Vrelbdot(1)*Vrelb(3))/(Vrelb(1)^2+Vrelb(3)^2);

% calculate standard atmosphere parameters
  [rho,P,T,a] = atmosphere(h);

% calculate Mach number
  M = V/a;

% calculate dynamic pressure
  qbar = rho*V^2/2;

% calculate aerodynamic forces and moments for a fixed wing aircraft
  [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,Omegab,deltaaero,qbar,M,deltaCGb,aircraft);

% calculate propulsion forces and moments
  [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft);

% calculate net force expressed in body frame
  Fnetb = Wb+Fab+Ftb;

% evaluate translational dynamics
  Vbdot = Fnetb/aircraft.m-cross(Omegab,Vb);

% calculate net force expressed in body frame
  Mnetb = Mab+Mtb;

% evaluate rotacional dynamics
  Omegabdot = aircraft.Ibinv*(Mnetb-cross(Omegab,aircraft.Ib*Omegab));

% assemble xdot
  xdot = [pedot;Phidot;Vbdot;Omegabdot];

% assemble y
  y = [V;alpha;beta];
end
