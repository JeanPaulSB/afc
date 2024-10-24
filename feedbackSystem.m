% feedbackSystem.m
% function to calculate feedback system state space model for the following model
%  Plant model:
%   xdot = A*x+B*u
%   y = C*x
%  Controller model
%   xcdot = Ac*xc+Bc*e
%   uc = Cc*xc+Dc*e
%  Interconnection equations
%   u = K1*ucl+K2*uc
%   e = K3*ucl+K4*y
% where
%  x : plant state vector
%  xdot : plant state vector derivative
%  u : plant input vector
%  xc : controller state vector
%  xcdot : controller state vector derivative
%  e : controller input vector (typically the error)
%  ucl : closed loop system input (typically some components of u and the setpoint)
% Usage:
%  [Acl,Bcl,Ccl,Dcl] = feedbackSystem(A,B,C,Ac,Bc,Cc,Dc,K1,K2,K3,K4)
% The obtained model is of the form:
%  [xdot;xcdot] = Acl*[x;xc]+Bcl*ucl
%  y = Ccl*[x;xc]+Dcl*ucl = x
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%


function [Acl,Bcl,Ccl,Dcl] = feedbackSystem(A,B,C,Ac,Bc,Cc,Dc,K1,K2,K3,K4)

np = size(A,1);
pp = size(C,1);
nc = size(Ac,1);
mi = size(K1,2);
Acl = [A+B*K2*Dc*K4*C  B*K2*Cc;Bc*K4*C Ac];
Bcl = [B*(K1+K2*Dc*K3);Bc*K3];
Ccl = [C  zeros(pp,nc)];
Dcl = zeros(pp,mi);
