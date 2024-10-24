% CourseProject.m
close all
pkg load control
format short g

% recall aircraft data
%Learjet24_cruiseMaxWeight_aircraft
Navion_aircraft

% set steady rectilinear flight conditions for Linearized model
V=aircraft.V
h=aircraft.h
deltaCGb = [0;0;0]

% linearize aircraft model around steady level rectilinear flight conditions
[E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)

% analyze aircraft dynamic modes
aircraftDynamicModes = analyzeAircraftDynamicModes(eigLon,eigLat,aircraft)

% calculate longitudinal transfer matrix
innames = {'deltat','ih','deltae'};
stnames = {'V','alpha','theta','q'};
outnames = {'V','alpha','theta','q'};
[aircraftLongitudinalModeltf,aircraftLongitudinalModelss] = getModelFromSSmatrices(Alonprime,Blonprime,eye(4),zeros(4,3),innames,stnames,outnames)

% extract transfer funtion from minus deltae to theta
tfFromMinusDeltaeToTheta = getTransferFunctionFromTransferMatrix(aircraftLongitudinalModeltf,3,3,-1);

% calculate lateral transfer matrix
innames = {'deltaa','deltar'};
stnames = {'beta','phi','ps','rs'};
outnames = {'beta','phi','ps','rs'};
[aircraftLateralModeltf,aircraftLateralModelss] = getModelFromSSmatrices(Alatprime,Blatprime,eye(4),zeros(4,2),innames,stnames,outnames)

% extract transfer funtion from minus deltaa to phi
tfFromMinusDeltaaToPhi = getTransferFunctionFromTransferMatrix(aircraftLateralModeltf,1,2,-1);

% bode plot for transfer funtion from minus deltae to theta
figure(3)
bode(tfFromMinusDeltaeToTheta);grid on;title('Bode plot for -theta(s)/deltae(s)')

% bode plot for transfer funtion from minus deltae to theta
figure(4)
bode(tfFromMinusDeltaaToPhi);grid on;title('Bode plot for -phi(s)/deltaa(s)')

% Recall control parameters
controlParameters

% longitudinal control
% pitch control
h = 5;
name = 'Pitch control';
pos = [0 0 1 1];
[clsystemtf,olsystemtf,controltf,closedLoopPoles] = closedLoopPIDcontrolResponse(tfFromMinusDeltaeToTheta,control.Kp_pitch,control.Ki_pitch,control.Kd_pitch,control.tau,h,name,pos)

% closed loop system with pitch control
[Acl,Bcl,Ccl,Dcl] = closedLoopSystemWithPIDcontroller(Alonprime,Blonprime,eye(4),-control.Kp_pitch,-control.Ki_pitch,-control.Kd_pitch,control.tau,3,3)

% calculate longitudinal transfer matrix with pitch control
innames = {'deltat','ih','pitchSP'};
stnames = {'V','alpha','theta','q','xcl'};
outnames = {'V','alpha','theta','q'};
[closedLoopLongitudinalModelWithPitchControltf,closedLoopLongitudinalModelWithPitchControlss] = getModelFromSSmatrices(Acl,Bcl,Ccl,Dcl,innames,stnames,outnames)

[tfFromDeltatToVwithPitchControl] = getTransferFunctionFromTransferMatrix(closedLoopLongitudinalModelWithPitchControltf,1,1,1)

% airspeed control
h = 6;
name = 'airspeed control';
[clsystemtf,olsystemtf,controltf,closedLoopPoles] = closedLoopPIDcontrolResponse(tfFromDeltatToVwithPitchControl,control.Kp_as,control.Ki_as,control.Kd_as,control.tau,h,name,pos)

% closed loop system with pitch and airspeed control
[Acl,Bcl,Ccl,Dcl] = closedLoopSystemWithPIDcontroller(Acl,Bcl,Ccl,control.Kp_as,control.Ki_as,control.Kd_as,control.tau,1,1);

% augment the model with altitude dynamics
n = size(Acl,1)
Acl = [Acl,zeros(n,1);0,-V,V,0,zeros(1,n-3)]
Bcl = [Bcl;0,0,0]
Ccl = [Ccl,zeros(4,1);zeros(1,n),1]
Dcl = [Dcl;0,0,0]

% calculate longitudinal transfer matrix with airspeed control augmented with altitude dynamics
innames = {'airspeedSP','ih','pitchSP'};
stnames = {'V','alpha','theta','q','xcl','xc2','h'};
outnames = {'V','alpha','theta','q','h'};
[closedLoopLongitudinalModelAugmentedWithAltitudetf,closedLoopLongitudinalModelWithPitchControlss] = getModelFromSSmatrices(Acl,Bcl,Ccl,Dcl,innames,stnames,outnames)

% obtain transfer funtion from pitchSP to h with pitch and airspeed control
[closedLoopTfFromPitchSPtoH] = getTransferFunctionFromTransferMatrix(closedLoopLongitudinalModelAugmentedWithAltitudetf,1,1,1)

% altitude control
h = 6;
name = ' altitude control';
[clsystemtf,olsystemtf,controltf,closedLoopPoles] = closedLoopPIDcontrolResponse(closedLoopTfFromPitchSPtoH,control.Kp_altitude,control.Ki_altitude,control.Kd_altitude,control.tau,h,name,pos)




%NOTAS DE LAS QUE ESPERO ACORDARME QUE SIGNIFICAN

%sistema tipo 0 -> error de estado estacionario =  1/((1+10)^(36.53/20))
%""36.53"" = valor mas a la izquierda grafica open loop blode plot

%margen de fase = (valor donde segunda grafica de open loop bode plot tiende a cero) y marco el valor de la fase
%figure;bode(olsystemtf);grid on

%margen de ganancia = valor pico de la primera grafica de closed loop step response
%figure;bode(clsystemtf);grid on
