% feedbackSystem.m
% function to calculate feedback system state space model for the following model
%  Plant model:
%   xdot = A*x+B*u
%   y = C*x
%  Controller model
%   uc = K*e
%  Interconnection equations
%   u = K1*ucl+K2*uc
%   e = K3*ucl+K4*y
% where
%  x : plant state vector
%  xdot : plant state vector derivative
%  u : plant input vector
%  e : controller input vector (typically the error)
%  ucl : closed loop system input (typically some components of u and the setpoint)
% Usage:
%  [Acl,Bcl,Ccl,Dcl] = feedbackSystemWithStaticFeedback(A,B,C,K,K1,K2,K3,K4)
% The obtained model is of the form:
%  xdot = Acl*x+Bcl*ucl
%  y = Ccl*x+Dcl*ucl = x
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%


function [Acl,Bcl,Ccl,Dcl] = feedbackSystemWithStaticFeedback(A,B,C,K,K1,K2,K3,K4)

np = size(A,1);
pp = size(C,1);
mi = size(K1,2);
Acl = A+B*K2*K*K4*C;
Bcl = B*(K1+K2*K*K3);
Ccl = C;
Dcl = zeros(pp,mi);
