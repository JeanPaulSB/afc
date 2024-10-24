% polysum.m
% function to sum two polynomials
% Usage:
%   Psum = polysum(P1,P2)
% where:
%   Psum : sum of polynomials
%   P1 : first polynomial
%   P2 : second polynomial
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function Psum = polysum(P1,P2)
  P1 = P1(find(P1,1,'first'):end);
  P2 = P2(find(P2,1,'first'):end);
  n1 = length(P1);
  n2 = length(P2);
  if n1~=n2
    if n1>n2
      P2 = [zeros(1,n1-n2),P2];
    else
      P1 = [zeros(1,n2-n1),P1];
    end
  end
  Psum = P1+P2;
end
