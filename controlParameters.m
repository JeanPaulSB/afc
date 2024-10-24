% controlParameters.m
% script to define aircraft control parameters for Automatic Flight Control course project
%
% Aircraft:
% Flight condition:
%
% Group:
% Students:
%
%
%
%

% Initial values
control.initialAltitude = 40000*0.3048;
control.initialHeading = 0*pi/180;

% trim values (u0 y x0)
control.ihtrim = 0; %(segundo valor u0)
control.deltaetrim = -0.0044046; %(tercer valor u0)
control.alphatrim = 0.0087476; %(tercer valor x0)
control.deltattrim = 0.33688; %(primer valor u0)

%control.ihtrim = 0.007554;
%control.deltaetrim = 0;
%control.alphatrim = 0.04852;
%control.deltattrim = 0.17753;

% Autopilot limits
control.climbRate = 1000*0.3048/60;
control.sinkRate = 500*0.3048/60;
control.maxAltitude = 18000*0.3048;
control.minAltitude = 0*0.3048;
control.turnRate = 3*pi/180;
control.Ts = 0.01;

% longitudinal control parameters
% pitch PID control parameters (inner loop)
control.Kp_pitch = 3.414;
control.Ki_pitch = 0;
control.Kd_pitch = 0.7;
% airspeed PID control parameters (outer loop)
control.Kp_as = 2;
control.Ki_as = 0.1; % 0.1*Kp como valor inicial
control.Kd_as = 0;
% altitude PID control parameters (outer loop)
control.Kp_altitude = 0;
control.Ki_altitude = 0;
control.Kd_altitude = 0;

% lateral control parameters
% roll PID parameters (inner loop)
control.Kp_roll = 0;
control.Ki_roll = 0;
control.Kd_roll = 0;
% sideslip PID parameters
control.Kp_sideslip = 0;
control.Ki_sideslip = 0;
control.Kd_sideslip = 0;
% heading PID parameters (outer loop)
control.Kp_heading = 0;
control.Ki_heading = 0;
control.Kd_heading = 0;

% derivative time constant
control.tau = 0.01;
