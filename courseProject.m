format short g

% call our aircraft data structure
% TODO

% set steady rectilinear flight conditions for Linearized model
V = aircraft.V
h = aircraft.h
deltaCGb = [0;0;0]

% linearize aircraft model around steady level rectilinear flight conditions
[E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)

% analyze aircraft dynamic modes
aircraftDynamicModes = analyzeAircraftDynamicModes(eigLon,eigLat,aircraft)

% calculate longitudinal transfer matrix
innames = {'deltat','ih','deltae'};
stnames = {'V','alpha','theta','q'};
outnames = {'V','alpha','theta','q'};
# TODO: to be completed
[aircraftLongitudinalModeltf,aircraftLongitudinalModelss] = getModelFromSSmatrices(Alonprime,Blonprime,eye(4),zeros(4,3),innames,stnames,outnames)
% extract transfer funtion from minus deltae to theta
[tfFromMinusDeltaeToTheta] = getTransferFunctionFromTransferMatrix(aircraftLongitudinalModeltf,3,3,-1);

% calculate lateral transfer matrix
innames = ['deltaa','deltar'];
stnames = ['beta','phi','ps','rs'];
outnames = ['beta','phi','ps','rs'];

# TODO: to be completed
[aircraftLateralModeltf,aircraftLateralModelss] = getModelFromSSmatrices(Alatprime,Blatprime,eye(4),zeros(4,2),innames,stnames,outnames)
tfFromMinusDeltaaToPhi = getTransferFunctionFromTransferMatrix(aircratLateralModeltf(1,2,-1))