clear all; close all; clc;
% Script dedicato al calcolo delle mappe per i motori elettrici AMK

%% Parametri del motore AMK

% Limiti massimi
mot.limit.Tmax = 21; % Coppia massima del motore [Nm] 
mot.limit.wmax = 20000; % Velocità massima del motore [rpm]
mot.limit.Pmax = 35000; % Potenza elettrica massima del motore [W]

% Valori nominali 
mot.rated.Tn = 9.8; % Coppia nominale [Nm]
mot.rated.wn = 12000; % Velocità nominale [rpm]
mot.rated.Pn = 12300; % Potenza nominale del motore [W]

%% Parametri del veicolo
vehicle.limit.Pmax = 80000; % Limite di potenza del veicolo [W]
vehicle.setup.torqueDistribution = 0.7; % Rear axle torque distribution []

% Parametri coppia di trazione
vehicle.setup.Torque = 21; % Limite di coppia impostato sul veicolo [Nm]
vehicle.setup.Power = 80000; % Limite di potenza impostato sul veicolo [W]
vehicle.setup.frontTorqueFactor = (1-vehicle.setup.torqueDistribution)/vehicle.setup.torqueDistribution;

% Parametri di Regen
vehicle.setup.RegenTorque = 18; % Limite di coppia impostato sul veicolo [Nm]
vehicle.setup.regenDistribution = 0.7;
vehicle.setup.rearRegenFactor = (1-vehicle.setup.regenDistribution)/vehicle.setup.regenDistribution;

% Parametri meccanici 
vehicle.param.wheelRadius = 0.2032; % Raggio nominale della ruota [m]
vehicle.param.gearRatio = 1/12.667; % Gear ratio between motor and wheel
%% Generazione delle mappe
nPoints = 51; % Numero di punti della mappa
nPointsPedals = 21;% Numero di punti in cui discretizzare acceleratore e freno
speedVector = linspace(0,mot.limit.wmax,nPoints); % Asse delle velocità utilizzato nelle mappe [rpm]
vehicleSpeedVector = speedVector * pi/30 * vehicle.param.wheelRadius * vehicle.param.gearRatio; % Velocità del veicolo [m/s]
TorqueRear = zeros(nPoints,nPointsPedals); % Vettore di coppia per asse rear [Nm]
throttleVector = linspace(0,1,nPointsPedals); % Discretizzazione acceleratore
brakeVector = throttleVector; % Discretizzazione freno

TorqueLimit = zeros(nPoints,1); % Vettore di coppia limite per i motori [Nm]
PowerLimit = zeros(nPoints,1); % Vettore di potenza limite per i motori [kW]
%% Calcolo delle mappe (HP: Distribuzione di coppia = Distribuzione di potenza)
PowerMotor_R = zeros(nPoints,nPointsPedals);

% Mappa accelerazione
for ispeed = 1:nPoints
    TorqueLimit(ispeed) = mot.limit.Pmax / (speedVector(ispeed) * pi/30);
    TorqueLimit(ispeed) = min(mot.limit.Tmax , TorqueLimit(ispeed));
    PowerLimit(ispeed) = TorqueLimit(ispeed) * (speedVector(ispeed) * pi/30);
    for ipedal = 1:nPointsPedals
        if speedVector(ispeed) ~= 0
            TorqueRear(ispeed,ipedal) = vehicle.setup.Power / (speedVector(ispeed) *(pi/30) *2*(1 + vehicle.setup.frontTorqueFactor));
            TorqueRear(ispeed,ipedal) = min(TorqueRear(ispeed,ipedal), vehicle.setup.Torque * throttleVector(ipedal));
        else
            TorqueRear(ispeed,ipedal) = vehicle.setup.Torque * throttleVector(ipedal);
        end
         PowerMotor_R(ispeed,ipedal) = speedVector(ispeed) * TorqueRear(ispeed,ipedal) * pi/30;
    end
end

TorqueFront = TorqueRear * vehicle.setup.frontTorqueFactor;
PowerMotor_F = PowerMotor_R * vehicle.setup.frontTorqueFactor;
PowerTOT = (PowerMotor_R + PowerMotor_F)*2;

vehicle.vicrt.frontMap = 1000 * TorqueFront; % Mappa di coppia al front [N*mm]
vehicle.vicrt.rearMap = 1000 * TorqueRear; % Mappa di coppia al rear [N*mm]

% Mappa regen
TorqueFrontRegen = zeros(nPoints,nPointsPedals); % Vettore di coppia di regen asse front [Nm]
PowerMotor_F_Regen = zeros(nPoints,nPointsPedals);

for ispeed = 1:nPoints
    for ipedal = 1:nPointsPedals
        if speedVector(ispeed) ~= 0
            TorqueFrontRegen(ispeed,ipedal) = mot.limit.Pmax / (speedVector(ispeed) * pi/30);
            TorqueFrontRegen(ispeed,ipedal) = min(TorqueFrontRegen(ispeed,ipedal), vehicle.setup.RegenTorque * throttleVector(ipedal));
        else
            TorqueFrontRegen(ispeed,ipedal) = vehicle.setup.RegenTorque *throttleVector(ipedal);
        end
        PowerMotor_F_Regen(ispeed,ipedal) = - speedVector(ispeed) * TorqueFrontRegen(ispeed,ipedal) * pi/30;
    end
end

TorqueRearRegen = TorqueFrontRegen * vehicle.setup.rearRegenFactor ;
PowerMotor_R_Regen = PowerMotor_F_Regen * vehicle.setup.rearRegenFactor;
PowerTOTRegen = 2 * (PowerMotor_R_Regen + PowerMotor_F_Regen);

vehicle.vicrt.frontMapRegen = 1000 * TorqueFrontRegen; % Mappa di coppia di regen al front [N*mm]
vehicle.vicrt.rearMapRegen = 1000 * TorqueRearRegen; % Mappa di coppia di regen al rear [N*mm]

%% Plot
figure(1)
plot(speedVector, TorqueFront(:,nPointsPedals), LineWidth=1.5);
hold on;
plot(speedVector, TorqueRear(:,nPointsPedals), LineWidth=1.5);
plot(speedVector, TorqueFrontRegen(:,nPointsPedals), LineWidth=1.5, LineStyle="--");
plot(speedVector, TorqueRearRegen(:,nPointsPedals), LineWidth=1.5, LineStyle="--")
plot(speedVector, TorqueLimit, LineWidth=1.5, LineStyle=":");
grid on;
ylim([0, mot.limit.Tmax*1.1]);
hold off;
legend({"Front Torque" "Rear Torque" "Front Regen" "Rear Regen" "Motor Torque Limit"});
xlabel("Motor speed [rpm]");
ylabel("Motor torque [Nm]");
title("Motors torque limits");
 
figure(2)
imagesc(throttleVector * 100 ,vehicleSpeedVector * 3.6, PowerTOT);
colorbar;
xlabel("Throttle [%]");
ylabel("Vehicle speed [km/h]");
title("Vehicle driving power map [kW]");

figure(3)
imagesc(brakeVector* 100,vehicleSpeedVector * 3.6, PowerTOTRegen);
colorbar;
xlabel("Brake [%]");
ylabel("Vehicle speed [km/h]");
title("Vehicle braking power map [kW]");

figure(4)
plot(speedVector, PowerMotor_F(:,nPointsPedals), LineWidth=1.5);
hold on;
plot(speedVector, PowerMotor_R(:,nPointsPedals), LineWidth=1.5);
plot(speedVector, abs(PowerMotor_F_Regen(:,nPointsPedals)), LineWidth=1.5, LineStyle="--");
plot(speedVector, abs(PowerMotor_R_Regen(:,nPointsPedals)), LineWidth=1.5, LineStyle="--");
plot(speedVector, PowerLimit, LineWidth=1.5, LineStyle=":");
grid on;
ylim([0, mot.limit.Pmax * 1.1]);
hold off;
legend({"Front Power" "Rear Power" "Front Regen Power" "Rear Regen Power"});
xlabel("Motor speed [rpm]");
ylabel("Motor power [kW]");
title("Motors power limits");
