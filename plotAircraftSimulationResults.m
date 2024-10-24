% plotAircraftSimulationResults.m
% function to plot fixed wing aircraft simulation results obtained with function simulateAircraft

function plotAircraftSimulationResults(t,X,Y,aircraft)
% plot airspeed
  figure(1)
  plot(t,Y(:,1)*3600/1852,'r-'); grid on; xlabel('t (s)'); ylabel ('V (kn)'); title(['Airspeed for ',aircraft.aircraftName]);

% plot angle of attack
  figure(2)
  plot(t,Y(:,2)*180/pi,'r-'); grid on; xlabel('t (s)'); ylabel ('\alpha (deg)'); title(['Angle of attack for ',aircraft.aircraftName]);

% plot angle of sideslip
  figure(3)
  plot(t,Y(:,3)*180/pi,'r-'); grid on; xlabel('t (s)'); ylabel ('\beta (deg)'); title(['Angle of sideslip for ',aircraft.aircraftName]);

% plot Euler angles
%   plot roll
  figure(4)
  plot(t,X(:,4)*180/pi,'r-'); grid on; xlabel('t (s)'); ylabel ('\phi (deg)'); title(['Roll for ',aircraft.aircraftName]);
%   plot pitch
  figure(5)
  plot(t,X(:,5)*180/pi,'r-'); grid on; xlabel('t (s)'); ylabel ('\theta (deg)'); title(['Pitch for ',aircraft.aircraftName]);
%   plot yaw / heading
  figure(6)
  plot(t,X(:,6)*180/pi,'r-'); grid on; xlabel('t (s)'); ylabel ('\psi (deg)'); title(['Yaw / Heading for ',aircraft.aircraftName]);

% plot angular velocity
%   plot roll rate
  figure(7)
  plot(t,X(:,10),'r-'); grid on; xlabel('t (s)'); ylabel ('p (rad/s)'); title(['Roll rate for ',aircraft.aircraftName]);
%   plot roll pitch rate
  figure(8)
  plot(t,X(:,11),'r-'); grid on; xlabel('t (s)'); ylabel ('q (rad/s)'); title(['Pitch rate for ',aircraft.aircraftName]);
%   plot roll rate
  figure(9)
  plot(t,X(:,12),'r-'); grid on; xlabel('t (s)'); ylabel ('r (rad/s)'); title(['Yaw rate for ',aircraft.aircraftName]);

% plot aircraft position
  figure(10)
  plot(X(:,2),X(:,1),'r-'); axis equal; grid on; xlabel('ye -> E (m)'); ylabel ('xe -> N (m)'); title(['Aircraft position for ',aircraft.aircraftName]);

% plot altitude
  figure(11)
  plot(t,-X(:,3)/0.3048,'r-'); grid on; xlabel('t (s)'); ylabel ('h (ft)'); title(['Altitude for ',aircraft.aircraftName]);
end
