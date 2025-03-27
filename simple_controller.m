%% Simple PID Controller Example
% This is a basic implementation of a PID controller in MATLAB

function output = simple_controller(setpoint, measured, Kp, Ki, Kd)
    % PID controller implementation
    % Inputs:
    %   setpoint - desired value
    %   measured - current measured value
    %   Kp - Proportional gain
    %   Ki - Integral gain
    %   Kd - Derivative gain
    % Output:
    %   output - controller output

    persistent integral
    persistent prev_error
    persistent prev_time
    
    % Initialize persistent variables
    if isempty(integral)
        integral = 0;
        prev_error = 0;
        prev_time = 0;
    end
    
    % Current time
    current_time = toc;
    
    % Calculate time step
    dt = current_time - prev_time;
    if dt <= 0
        dt = 0.01; % Default if first run or timer reset
    end
    
    % Calculate error
    error = setpoint - measured;
    
    % Proportional term
    P = Kp * error;
    
    % Integral term
    integral = integral + error * dt;
    I = Ki * integral;
    
    % Derivative term
    derivative = (error - prev_error) / dt;
    D = Kd * derivative;
    
    % PID output
    output = P + I + D;
    
    % Update previous values
    prev_error = error;
    prev_time = current_time;
end
