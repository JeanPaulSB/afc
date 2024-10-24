% H.m
% function to calculate Hmatrix such that Phidot = Hmatrix*Omegab
% usage
%   Hmatrix = H(Phi)
% where
%   Phi = [phi;theta;psi] : Euler angles (rad)
%     with
%       phi : roll (rad)
%       theta :  pitch (rad)
%       psi : yaw or heading (rad)
%

function Hmatrix = H(Phi)
% extract Euler angles from Phi
  phi = Phi(1,1);
  theta = Phi(2,1);

% calculate cosine and sine of Euler angles
  cphi = cos(phi);
  sphi = sin(phi);
  ctheta = cos(theta);
  ttheta = tan(theta);

% calculate Hmatrix
  Hmatrix = [1   ttheta*sphi   ttheta*cphi;
            0   cphi          -sphi;
            0   sphi/ctheta   cphi/ctheta;];
end
