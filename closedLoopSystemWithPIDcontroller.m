% closedLoopSystemWithPIDcontroller.m
% function to calculate closed loop system state space model for given plant 
% with PID control loop to control plant output out from plant input in.
% The obtained model is of the form:
%  [xdot;xcdot] = Acl*[x;xc]+Bcl*ucl
%  y = Ccl*[x;xc]+Dcl*ucl = x
% Components of ucl correspond to plant inputs except for input in which is 
% the PID controller set point. 
% Usage
%   [Acl,Bcl,Ccl,Dcl] = closedLoopSystemWithPIDcontroller(A,B,C,Kp,Ki,Kd,tau,in,out)
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function [Acl,Bcl,Ccl,Dcl] = closedLoopSystemWithPIDcontroller(A,B,C,Kp,Ki,Kd,tau,in,out)

% determine order (n), number of inputs (m), and number of outputs (p) of plant
n = size(A,1);
m = size(B,2);
p = size(C,1);
%  Interconnection equations
%   u = K1*ucl+K2*uc
%   e = K3*ucl+K4*y
K1 = eye(m);
K1(in,in) = 0;
K2 = zeros(m,1);
K2(in) = 1;
K3 = zeros(1,m);
K3(in) = 1;
K4 = zeros(1,p);
K4(out) = -1;

if Ki==0 && Kd==0
  % calculate closed loop system state space model
  [Acl,Bcl,Ccl,Dcl] = feedbackSystemWithStaticFeedback(A,B,C,Kp,K1,K2,K3,K4);
else
  if Ki~=0 && Kd~=0 && tau~=0
    % calculate state space model for PID controller
    [Ac,Bc,Cc,Dc] = PIDcontrollerss(Kp,Ki,Kd,tau);
  elseif Kd~=0 && tau~=0
    % calculate state space model for PD controller
    [Ac,Bc,Cc,Dc] = PDcontrollerss(Kp,Kd,tau);
  elseif Ki~=0
    % calculate state space model for PI controller
    [Ac,Bc,Cc,Dc] = PIcontrollerss(Kp,Ki);
  end
  % calculate closed loop system state space model
  [Acl,Bcl,Ccl,Dcl] = feedbackSystem(A,B,C,Ac,Bc,Cc,Dc,K1,K2,K3,K4);
end

