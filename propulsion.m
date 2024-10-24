% propulsion.m
% function to calculate propulsion system forces and moments
% usage
%   function [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft)
% where
%   deltat : propulsion system control (0<=deltat<=1)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure
%   Ftb: net propulsion system force (N)
%   Mtb: net propulsion system moment (Nm)
%

function [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft)
% calcualte net propulsion system force
  Ftb = [1;0;0]*aircraft.Tmax*deltat; %Propulsion system force vector will in the x axis direction
  Mtb = -cross(deltaCGb,Ftb);
end
