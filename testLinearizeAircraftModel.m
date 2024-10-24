%testLinearizeAircraftModel.m
% Script to open LinearizeAircraftMondel and analyze aircraft dynamic model
% for a fixed wing

clear all
clc

% recall aircraft data structure
Navion_aircraft

% set flight conditions for linearized model
V = aircraft.V;
h = aircraft.h;

deltaCGb = [0;0;0];

%calculate aircraft linerized model and longitudinal end lateral
%eigenvalues

[E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)


%aircraft = analyzeAircraftDynamicModes(eigLon,eigLat,aircraft);

%dynamicModes = aircraft.dynamicsModes
