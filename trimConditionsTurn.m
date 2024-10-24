% trimConditionsTurn.m
% function to calculate steady turn flight for a fixed wing aircraft
% usage
%   [phi,theta,gamma,alpha,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsTurn(V,h,Vvert,turnRate,deltaCGb,aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   turnRate : turn rate (rad/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   y : value of funtion to be minimized to obtain steady rectilinear flight conditions
%   phi : roll (rad)
%   theta : pitch (rad)
%   gamma : flight path angle
%   alpha : angle of attack (rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence (rad)
%   deltae : elevator defflection (rad)
%   deltaa : aileron (rad)
%   deltar : rudder (rad)
%   fval : value of ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft) in solution of optimization
%   flag : flag to indicate how the numerical optimization method  exited

function [phi,theta,gamma,alpha,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsTurn(V,h,Vvert,turnRate,deltaCGb,aircraft)
% calculate steady level rectilinear flight conditions
  if aircraft.Cmih ~= 0 && aircraft.CMdeltae ~= 0
    [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb,aircraft)
  else
    ihTrimRect = 0;

%   flight path angle
      gamma = asin(Vvert/V);
  end

% set initial guess for numerical optimization method
  phi0 = atan(V*turnRate/aircraft.g);
  theta0 = gamma;
  alpha0 = 0*pi/180;
  deltat0 = 0.5;
  pitchControl0 = 0*pi/180;
  deltaa0 = 0*pi/180;
  deltar0 = 0*pi/180;
  Xi0 = [phi0;theta0;alpha0;deltat0;pitchControl0;deltaa0;deltar0];

% set lower bound for Xi
  phimin = -60*pi/100;
  thetamin  = -25*pi/10;
  alphamin = -5*pi/180;
  deltatmin = 0;
  pitchControlmin = -20*pi/180;
  deltaamin = -20*pi/180;
  deltarmin = -20*pi/180;
  lb = [phimin;thetamin;alphamin;deltatmin;pitchControlmin;deltaamin;deltarmin];

% set upper bound for Xi
  phimax = -60*pi/100;
  thetamax  = 32*pi/10;
  alphamax = 12*pi/180;
  deltatmax = 1;
  pitchControlmax = 20*pi/180;
  deltaamax = 20*pi/180;
  deltarmax = 20*pi/180;
  ub = [phimax;thetamax;alphamax;deltatmax;pitchControlmax;deltaamax;deltarmax];

% set numerical optimization parameters
  maxiter = 1e5;
  tol = 1e-9;

  if isOctave()
    [Xitrim,fval,flag] = sqp(Xi0,@(Xi) ftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft),[],[],lb,ub,maxiter,tol);
  else
    options = optimoptions('fmincon','Display','off','Algorithm','sqp',...
              'MaxIter',maxiter,'TolX',tol,'TolFun',tol);
    [Xitrim,fval,flag] = fmincon(@(Xi) ftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
  end

% set variables for solution in steady flight conditions
  phi = Xitrim(1);
  theta = Xitrim(2);
  alpha = Xitrim(3);
  deltat = Xitrim(4);
  pitchControl = Xitrim(5);
  deltaa = Xitrim(6);
  deltar = Xitrim(7);
  if aircraft.Cmdeltae ~= 0
    ih = ihTrimRect;
    deltae = pitchControl;
  else
    ih = pitchControl;
    deltae = 0;
  end
end
