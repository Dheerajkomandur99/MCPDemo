%% Control System Demo
% This file demonstrates basic control system concepts in MATLAB
% Created for MCPDemo repository

%% System Definition
% Define a simple second-order system
s = tf('s');
G = 1/(s^2 + 2*s + 1);

% Display the transfer function
disp('Open-loop transfer function:');
G

%% Step Response Analysis
% Analyze the step response of the open-loop system
figure(1);
step(G);
title('Step Response of Open-Loop System');
grid on;

%% PID Controller Design
% Define a simple PID controller
Kp = 2;
Ki = 1;
Kd = 0.5;

C = Kp + Ki/s + Kd*s;
disp('PID Controller:');
C

% Calculate the closed-loop transfer function
T = feedback(C*G, 1);
disp('Closed-loop transfer function:');
T

%% Closed-loop Analysis
% Analyze the step response of the closed-loop system
figure(2);
step(G, T);
title('Step Response Comparison');
legend('Open-Loop', 'Closed-Loop with PID');
grid on;

%% Frequency Response
% Generate Bode plot
figure(3);
bode(G, T);
title('Frequency Response');
legend('Open-Loop', 'Closed-Loop with PID');
grid on;

%% Root Locus Analysis
% Generate root locus plot for the system with proportional control
figure(4);
rlocus(G);
title('Root Locus');
grid on;

%% Stability Analysis
% Calculate gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(G);
fprintf('Gain Margin: %f dB\n', 20*log10(Gm));
fprintf('Phase Margin: %f degrees\n', Pm);

%% Simulation Parameters
% Define simulation time
t = 0:0.01:10;

% Define input signal
r = ones(size(t)); % Step input

% Simulate open-loop response
[y_ol, t_ol] = lsim(G, r, t);

% Simulate closed-loop response
[y_cl, t_cl] = lsim(T, r, t);

%% Plot Simulation Results
figure(5);
plot(t, r, 'k--', t_ol, y_ol, 'b-', t_cl, y_cl, 'r-', 'LineWidth', 2);
title('System Response to Step Input');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Reference Input', 'Open-Loop Response', 'Closed-Loop Response');
grid on;

%% End of demo
disp('Control system demo completed successfully.');
