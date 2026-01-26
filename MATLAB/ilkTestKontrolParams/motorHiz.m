%% System Model and Analysis

clc, clear, close all

% J = 0.01;
% b = 0.1;
% K = 0.01;
% R = 1;
% L = 0.5;

s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2); % TF of RPM/Voltage

P_motor_ss = ss(P_motor); % SS of RPM/Voltage

rP_motor = 0.1/(0.5*s+1); % Pole at s = -2, SS value 0.1
% Used instead of actual TF to make the system first-order

%% Analysis
% linearSystemAnalyzer('step', P_motor, 0:0.1:5)

%% Control

% PID
% Kp = 100;
% Ki = 200;
% Kd = 10;

C_pid = pid(Kp,Ki,Kd);
sys_cl_pid = feedback(C_pid*P_motor,1);

% step(sys_cl_pid,[0:0.1:5])
% title('PID Control')

% Root Locus
% controlSystemDesigner('rlocus', P_motor)

% Bode
controlSystemDesigner('rlocus', P_motor)