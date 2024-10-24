% polyapprox.m
% function to approximate polynomial with given relative tolerance, making zero 
% terms below that relative tolerance respect to magnitude of polynomial
% Usage:
%   y = polyapprox(x,reltol)
% where:
%   y : approximate polynomial
%   x : original polynomial
%   reltol : relative tolerance
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function y = polyapprox(x,reltol)
  x = approx(x,reltol);
  y = x(find(x,1,'first'):end);
end
