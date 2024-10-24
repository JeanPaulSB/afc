% PDcontrollerss.m
% function to calculate state space model for PD controller with transfer function
%  C(s) = Kp + Kd*s/(tau*s + 1)
% Usage
%  [Ac,Bc,Cc,Dc] = PIDcontrollerss(Kp,Kd,tau)
% where the controller is represented in state space as
%  xcdot = Ac*xc+Bc*e
%  uc = Cc*xc+Dc*e
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%


function [Ac,Bc,Cc,Dc] = PDcontrollerss(Kp,Kd,tau)

Ac = -1/tau;
Bc = 1/tau;
Cc = -Kd/tau;
Dc = Kp+Kd/tau;
