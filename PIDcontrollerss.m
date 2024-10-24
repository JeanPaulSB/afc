% PIDcontrollerss.m
% function to calculate state space model for PID controller with transfer function
%  C(s) = Kp + Ki/s + Kd*s/(tau*s + 1)
% Usage
%  [Ac,Bc,Cc,Dc] = PIDcontrollerss(Kp,Ki,Kd,tau)
% where the controller is represented in state space as
%  xcdot = Ac*xc+Bc*e
%  uc = Cc*xc+Dc*e
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%


function [Ac,Bc,Cc,Dc] = PIDcontrollerss(Kp,Ki,Kd,tau)

Ac = [0 0;0 -1/tau];
Bc = [1;1/tau];
Cc = [Ki -Kd/tau];
Dc = Kp+Kd/tau;
