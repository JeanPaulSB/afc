% analyzeAircraftDynamicModes.m
% function to analyze longitudinal and lateral aircraft dynamic modes
% usage
%   analyzeAircraftDynamicModes(eigLon,eigLat)
% where
%   eigLon : longitudinal eigenvalues
%   eigLat : lateral eigenvalues%
%

function aircraftDynamicModes = analyzeAircraftDynamicModes(eigLon,eigLat,aircraft)
  aircraftDynamicModes.eigLon = eigLon;
  eig1 = eigLon(1);
  eig2 = eigLon(2);
  aircraftDynamicModes.phugoid.eig1 = eig1;
  aircraftDynamicModes.phugoid.eig2 = eig2;
  wn = sqrt(abs(eig1*eig2));
  aircraftDynamicModes.phugoid.wn = wn;
  aircraftDynamicModes.phugoid.zeta = -(real(eig1+eig2))/(2*wn);
  if ~isreal(eig1)
    wd = abs(imag(eig1));
    aircraftDynamicModes.phugoid.wd = wd;
    aircraftDynamicModes.phugoid.T = 2*pi/wd;
  end
  aircraftDynamicModes.phugoid.tau = -1/real(eig1);
  eig1 = eigLon(3);
  eig2 = eigLon(4);
  aircraftDynamicModes.shortPeriod.eig1 = eig1;
  aircraftDynamicModes.shortPeriod.eig2 = eig2;
  wn = sqrt(abs(eig1*eig2));
  aircraftDynamicModes.shortPeriod.wn = wn;
  aircraftDynamicModes.shortPeriod.zeta = -(real(eig1+eig2))/(2*wn);
  if ~isreal(eig1)
    wd = abs(imag(eig1));
    aircraftDynamicModes.shortPeriod.wd = wd;
    aircraftDynamicModes.shortPeriod.T = 2*pi/wd;
  end
  aircraftDynamicModes.shortPeriod.tau = -1/real(eig1);
  aircraftDynamicModes.eigLat = eigLat;
  eig1 = eigLat(1);
  aircraftDynamicModes.spiral.eig = eig1;
  aircraftDynamicModes.spiral.tau = -1/real(eig1);
  eig1 = eigLat(2);
  eig2 = eigLat(3);
  aircraftDynamicModes.dutchRoll.eig1 = eig1;
  aircraftDynamicModes.dutchRoll.eig2 = eig2;
  wn = sqrt(abs(eig1*eig2));
  aircraftDynamicModes.dutchRoll.wn = wn;
  aircraftDynamicModes.dutchRoll.zeta = -(real(eig1+eig2))/(2*wn);
  if ~isreal(eig1)
    wd = abs(imag(eig1));
    aircraftDynamicModes.dutchRoll.wd = wd;
    aircraftDynamicModes.dutchRoll.T = 2*pi/wd;
  end
  aircraftDynamicModes.dutchRoll.tau = -1/real(eig1);
  eig1 = eigLat(4);
  aircraftDynamicModes.roll.eig = eig1;
  aircraftDynamicModes.roll.tau = -1/real(eig1);

  % plot longitudinal eigenvalues
  figure(1)
  plot(real(eigLon),imag(eigLon),'rx');sgrid;xlabel('Re');ylabel('Im');
  title(['Longitudinal eigenvalues for ',aircraft.aircraftName])

  % plot lateral eigenvalues
  figure(2)
  plot(real(eigLat),imag(eigLat),'rx');sgrid;xlabel('Re');ylabel('Im');
  title(['Lateral eigenvalues for ',aircraft.aircraftName])
end
