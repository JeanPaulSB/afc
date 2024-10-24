% getModelFromSSmatrices.m
% function to get state space model and transfer matrix model from state space matrices
% Usage:
%   [modelss,modeltf] = getModelFromSSmatrices(A,B,C,D,innames,stnames,outnames)
% where:
%   A,B,C,D : state space matrices for state space model of the form xdot=A*x+B*u, y=C*x+D*u
%   innames : cell array with list of system inputs names 
%   stnames : cell array with list of system states names
%   outnames : cell array with list of system outputs names
%   modelss : state space model
%   modeltf : transfer matrix model
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function [modeltf,modelss] = getModelFromSSmatrices(A,B,C,D,innames,stnames,outnames)
  reltol = 1e-6;
  if isOctave()
    modelss = ss(A,B,C,D,'inname',innames,'stname',stnames,'outname',outnames);
  else
    modelss = ss(A,B,C,D,'InputName',innames,'StateName',stnames,'OutputName',outnames);
  end
  modeltf = tfapprox(tf(modelss),reltol);
end
