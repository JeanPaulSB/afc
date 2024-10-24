% trimConditionsRect.m
% function to calculate steady rectilinear flight for a fixed wing aircraft
% usage
%   [theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb,aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   y : value of funtion to be minimized to obtain steady rectilinear flight conditions
%   theta : pitch (rad)
%   gamma : flight path angle
%   alpha : angle of attack (rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence (rad)
%   deltae : elevator defflection (rad)
%   fval : value of ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft) in solution of optimization
%   flag : flag to indicate how the numerical optimization method  exited

function [theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb,aircraft)
% set initial guess for numerical optimization method
  alpha0 = 0*pi/180;
  deltat0 = 0.5;
  pitchControl0 = 0*pi/180;
  Xi0 = [alpha0;deltat0;pitchControl0];

% set lower bound for Xi
  alphamin = -5*pi/180;
  deltatmin = 0;
  pitchControlmin = -20*pi/180;
  lb = [alphamin;deltatmin;pitchControlmin];

% set upper bound for Xi
  alphamax = 12*pi/180;
  deltatmax = 1;
  pitchControlmax = 20*pi/180;
  ub = [alphamax;deltatmax;pitchControlmax];

% set numerical optimization parameters
  maxiter = 1e5;
  tol = 1e-9;

  if isOctave()
    [Xitrim,fval,flag] = sqp(Xi0,@(Xi) ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft),[],[],lb,ub,maxiter,tol);
  else
    options = optimoptions('fmincon','Display','off','Algorithm','sqp',...
              'MaxIter',maxiter,'TolX',tol,'TolFun',tol);
    [Xitrim,fval,flag] = fmincon(@(Xi) ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
  end

% set variables for solution in steady flight conditions
  alpha = Xitrim(1);
  gamma = asin(Vvert/V);
  theta = gamma+alpha;
  deltat = Xitrim(2);
  pitchControl = Xitrim(3);
  if aircraft.Cmih ~= 0
    ih = pitchControl;
    deltae = 0;
  else
    ih = 0;
    deltae = pitchControl;
  end
end
