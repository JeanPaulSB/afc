% approx.m
% function to approximate matrix with given relative tolerance, making zero 
% terms below that relative tolerance respect to magnitude of matrix
% Usage:
%   y = approx(x,reltol)
% where:
%   y : approximate matrix
%   x : original matrix
%   reltol : relative tolerance
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function y = approx(x,reltol)
  xnorm = norm(x);
  if xnorm~=0
    [n,m] = size(x);
    y = zeros(n,m);
    for i=1:n
      for j=1:m
        if abs(x(i,j)/xnorm)<reltol
          y(i,j) = 0;
        else
          y(i,j) = x(i,j);
        end
      end
    end
  else
    y=x;
  end
end
