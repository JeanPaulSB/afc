% tfapprox.m
% function to approximate trasfer function with given relative tolerance, making zero 
% terms below that relative tolerance respect to magnitude of polynomial
% Usage:
%   tfy = tfapprox(tfx,reltol)
% where:
%   tfy : approximate transfer function
%   tfx : original transfer function
%   reltol : relative tolerance
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function tfy = tfapprox(tfx,reltol)
  [n,m] = size(tfx.num);
  for i=1:n
    for j=1:m
      tfx.num{i,j} = polyapprox(tfx.num{i,j},reltol);
      tfx.den{i,j} = polyapprox(tfx.den{i,j},reltol);
    end
  end
  tfy = tfx;
end
