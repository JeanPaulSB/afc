% PIcontrollerss.m
% function to calculate state space model for PID controller with transfer function
%  C(s) = Kp + Ki/s
% Usage
%  [Ac,Bc,Cc,Dc] = PIcontrollerss(Kp,Ki)
% where the controller is represented in state space as
%  xcdot = Ac*xc+Bc*e
%  uc = Cc*xc+Dc*e
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%


function [Ac,Bc,Cc,Dc] = PIcontrollerss(Kp,Ki)

Ac = 0;
Bc = 1;
Cc = Ki;
Dc = Kp;
