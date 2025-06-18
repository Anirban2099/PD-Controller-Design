clc;
clear all;
close all;

% System Parameters
Jl = 10;
Bl = 1;
k = 30;
Jm = 2;
Bm = 0.5;

% Transfer Functions of Load and Motor
Pl = [Jl Bl k];  % Denominator of load
Pm = [Jm Bm k];  % Denominator of motor

% Define transfer functions
sys1 = tf(1, Pm);  % Motor dynamics: 1 / (Jm*s^2 + Bm*s + k)
sys2 = tf(1, Pl);  % Load dynamics: 1 / (Jl*s^2 + Bl*s + k)

% Series connection of motor and load
series_sys = series(sys1, sys2);

% Open-loop transfer function with spring stiffness as gain in feedback
% Feedback gain is positive (spring torque opposes deflection)
OLTF = feedback(series_sys, k);

% Plot step response
figure;
step(OLTF);
title('Step Response of the System with Joint Flexibility');
xlabel('Time (s)');
ylabel('Output');
grid on;
