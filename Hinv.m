% Hinv.m
% function to calculate Hinv matrix such that Omegab = Hinv(Phi)*Phidot
% usage
%   Hinvmatrix = Hinv(Phi)
% where
%   Phi = [phi;theta;psi] : Euler angles (rad)
%     with
%       phi : roll (rad)
%       theta :  pitch (rad)
%       psi : yaw or heading (rad)
%    Hinvmatrix : matrix such that omegab = Hinv(Phi)*Phidot
%

function Hinvmatrix = Hinv(Phi)
% extract Euler angles from Phi
  phi = Phi(1,1);
  theta = Phi(2,1);
  psi = Phi(2,1);

% calculate cosine and sine of Euler angles
  cphi = cos(phi);
  sphi = sin(phi);
  ctheta = cos(theta);
  stheta = sin(theta);

% calculate Hmatrix
  Hinvmatrix = [1  0       -stheta;
                0  cphi    ctheta*sphi;
                0  -sphi   ctheta*cphi];
end
