clc
close all
clear
%% Launcher Torque Vectoring 2025 - alberto.marconato@raceup.it

% Property of Alberto Marconato, head of Vehicle Dynamics department
% In this script will be defined all the parameters used by the control
% system

%% Dati del sistema 

m = 272; % Vehicle mass [kg]
ms = m - 8.135 * 4; % Vehicle sprung mass [kg]
wb = 0.45; % Front weight balance []
w = 1.535; % Vehicle wheelbase [m]
a = (1-wb)*w; % Front axle distance from CoG [m]
b = w-a; % Rear axle distance from CoG [m]

g = 9.81; % Gravitational acceleration [m/s^2]

tFront = 1.23; % Front track [m]
tRear = 1.2; % Rear track [m]

Izz = 112.4; % Vehicle yaw moment of inertia [kg*m^2]

Cf = 515.7027*180/pi; % Front axle static cornering stiffness [N/rad]
Cr = 630.3033*180/pi;  % Rear axle static cornering stiffness [N/rad]

load("steerFront.mat"); % Kinematic steering LUT [deg to deg]

Ts = 0.001; % System simulation sample time [s]

%% PI controller design

nominalWheelRadius = 0.2032; % Nominal wheel radius [m]
tau = 12.667; % Motor to wheel reduction ratio []
maxMotorTorque = 21; % Maximum motor torque [Nm]
maxForceAtWheel = maxMotorTorque * tau / nominalWheelRadius; % Maximum force at wheel [N]
maxYMC = maxForceAtWheel *0.4 * (tFront + tRear); % Maximum YM control torque achievable [Nm]

% PIdesign function call
% 
BP = 1*2*pi; % Desired bandwidth [rad/s]
phaseMargin = deg2rad(90); % Required phase margin at BP rad/s [rad]

Tc = 1e-3; % System control time [s]

% Low Pass filter for SWA input

freqLP = 25; % Cut frequency for low pass filter [Hz]
omegaLP = 2*pi*freqLP; % Cut frequency for low pass filter [rad/s]

%% Start maxperformance simulation

taregtKUS = 0.02; % Target Understeer gradient [deg/g]
% 
% vicrt_outputprefix = "SGe-08_crc_simulink"; 
% 
% vicrt_inputfile = uigetfile("svm.xml"); % Import sim file

%% PARAMETRI AGGIUNTI PER FUNZIONAMENTO SUL SIMULATORE

Pmax = 80000; % Limite di potenza impostato sul veicolo [W]
torqueDistribution = 0.7; % Rear axle torque distribution []
wmax = 20000; % Limite di velocità dei motori [rpm]

% Parametri coppia di trazione
Torque = 21; % Limite di coppia impostato sul veicolo [Nm]
frontTorqueFactor = (1-torqueDistribution)/torqueDistribution; % Fattore di scala per coppia al front

% Parametri di Regen
RegenTorque = 18; % Limite di coppia impostato sul veicolo [Nm]
regenDistribution = 0.7;
rearRegenFactor = (1-regenDistribution)/regenDistribution; % Fattore di scala per coppia al rear
%% Generazione delle mappe
nPoints = 51; % Numero di punti della mappa
nPointsPedals = 21;% Numero di punti in cui discretizzare acceleratore e freno
speedVector = linspace(0,wmax,nPoints); % Asse delle velocità utilizzato nelle mappe [rpm]
throttleVector = linspace(0,1,nPointsPedals); % Discretizzazione acceleratore
brakeVector = throttleVector; % Discretizzazione freno

% Caricamento delle mappe
load("torqueMaps.mat");