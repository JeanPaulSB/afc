% closedLoopPIDcontrolResponse.m
% function to analyze closed loop response with PID control
% Usage:
%   [clsystemtf,olsystemtf,controltf,closedLoopPoles] = closedLoopPIDcontrolResponse(planttf,Kp,Ki,Kd,tau,h,name,pos)
% where:
%   clsystemtf : close loop system transfer function
%   olsystemtf : open loop system transfer function
%   controltf : control transfer function
%   closedLoopPoles : closed loop poles
%   planttf : plant transfer function
%   Kp : proportional gain
%   Ki : integral gain
%   Kd : derivative gain
%   tau : derivative time constant
%   closedLoopPoles : vector with closed loop poles
%   h : figure handle
%   name : string with figure name
%   pos = [posx posy, width, height] : position and size of the figure (normalized values between 0 and 1) 
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function [clsystemtf,olsystemtf,controltf,closedLoopPoles] = closedLoopPIDcontrolResponse(planttf,Kp,Ki,Kd,tau,h,name,pos)
  [numPlant,denPlant] = tfdata(planttf);
  numPlant = numPlant{1,1};
  denPlant = denPlant{1,1};
  plantZeros = roots(numPlant);
  plantPoles = roots(denPlant);
  if length(numPlant)>length(denPlant)
    disp('ERROR: planttf must be a proper transfer function!')
    clsystemtf = [];
    olsystemtf = [];
    controltf = [];
    closedLoopPoles = [];
    return;
  end
  if Ki==0 && Kd==0
    controltf = tf(Kp);
  elseif Ki~=0 && Kd==0
    controltf = tf([Kp Ki],[1 0]);
  elseif Ki==0 && Kd~=0
    controltf = tf(Kp)+tf([Kd 0],[tau 1]);
  else
    controltf = tf([Kp Ki],[1 0])+tf([Kd 1],[tau 1]);
  end
  [numControl,denControl] = tfdata(controltf);
  numControl = numControl{1,1};
  denControl = denControl{1,1};
  controlZeros = roots(numControl);
  controlPoles = roots(denControl);
  numOLsystem = conv(numControl,numPlant);
  denOLsystem = conv(denControl,denPlant);
  numOLsystem = numOLsystem/denOLsystem(1);
  denOLsystem = denOLsystem/denOLsystem(1);
  olsystemtf = tf(numOLsystem,denOLsystem);
  numCLsystem = conv(numControl,numPlant);
  denCLsystem = polysum(conv(denControl,denPlant),conv(numControl,numPlant));
  numCLsystem = numCLsystem/denCLsystem(1);
  denCLsystem = denCLsystem/denCLsystem(1);
  clsystemtf = tf(numCLsystem,denCLsystem);
  numManipulatedVariable = conv(numControl,denPlant);
  denManipulatedVariable = polysum(conv(denControl,denPlant),conv(numControl,numPlant));
  numManipulatedVariable = numManipulatedVariable/denManipulatedVariable(1);
  denManipulatedVariable = denManipulatedVariable/denManipulatedVariable(1);
  manipulatedVariabletf = tf(numManipulatedVariable,denManipulatedVariable);
  closedLoopPoles = pole(clsystemtf);
  figureWithNameAndPosition(h,name,pos);
  [mag,phase,w] = bode(olsystemtf);
  if ~isOctave()
    mag = reshape(mag,length(mag),1);
    phase = reshape(phase,length(phase),1);
  end
  magdB = 20*log10(mag);
  subplot(421);semilogx(w,magdB,'Linewidth',2);grid on;xlabel('w (rad/s)');ylabel('Magnitude (dB)');title('Open loop bode plot')
  subplot(423);semilogx(w,phase,'Linewidth',2);grid on;xlabel('w (rad/s)');ylabel('Phase (deg)')
  [mag,phase,w] = bode(clsystemtf);
  if ~isOctave()
    mag = reshape(mag,length(mag),1);
    phase = reshape(phase,length(phase),1);
  end
  magdB = 20*log10(mag);
  subplot(425);semilogx(w,magdB,'Linewidth',2);grid on;xlabel('w (rad/s)');ylabel('Magnitude (dB)');title('Closed loop bode plot')
  subplot(427);semilogx(w,phase,'Linewidth',2);grid on;xlabel('w (rad/s)');ylabel('Phase (deg)')
  subplot(222);rlocus(olsystemtf);legend off;grid on;title('Root locus');hold on;
  plot(real(plantPoles),imag(plantPoles),'xb','Linewidth',2);plot(real(plantZeros),imag(plantZeros),'ob','Linewidth',2);
  plot(real(controlPoles),imag(controlPoles),'xr','Linewidth',2);plot(real(controlZeros),imag(controlZeros),'or','Linewidth',2);
  plot(real(closedLoopPoles),imag(closedLoopPoles),'+m','Linewidth',2);
  [y,t] = step(clsystemtf);
  subplot(426);plot(t,y,'Linewidth',2);grid on;xlabel('t (s)');ylabel('y(t)');title('Closed loop step response')
  [y,t] = step(manipulatedVariabletf);
  subplot(428);plot(t,y,'Linewidth',2);grid on;xlabel('t (s)');ylabel('u(t)');title('')
end
