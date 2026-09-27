% generate_mems_data.m
% Simulates a moving joint and injects real-world MEMS noise characteristics.
% Exports the noisy data to a C header file for embedded EKF testing.

%% 1. Define the Physical System
fs = 100;               % 100 Hz sampling frequency (Standard for MEMS)
dt = 1/fs;              % Time step
t = (0:dt:5-dt);        % 5 seconds of simulation
N = length(t);

% Ground Truth: The joint is sweeping back and forth (sine wave)
% Amplitude: 30 degrees, Frequency: 0.5 Hz
true_angle = 30 * sin(2*pi*0.5*t); 
true_rate = 30 * (2*pi*0.5) * cos(2*pi*0.5*t); % Derivative of angle

%% 2. Inject Real-World MEMS Physics (Noise & Drift)
% Accelerometer: Noisy but absolute (no drift)
accel_noise_std = 3.0; % High vibration noise
accel_meas = true_angle + accel_noise_std * randn(1, N);

% Gyroscope: Clean short-term, but suffers from Bias Drift (Random Walk)
gyro_noise_std = 0.2;  % Low immediate noise
gyro_bias = 0.05 * cumsum(randn(1, N)); % The dreaded physical MEMS drift
gyro_meas = true_rate + gyro_noise_std * randn(1, N) + gyro_bias;

%% 3. Plot the Physics to Verify
figure;
plot(t, true_angle, 'k', 'LineWidth', 2); hold on;
plot(t, accel_meas, 'r.', 'MarkerSize', 4);
plot(t, cumsum(gyro_meas)*dt, 'b--'); % Integrating gyro to show drift
legend('True Angle', 'Noisy Accelerometer', 'Integrated Gyroscope (Drifting)');
title('MEMS Sensor Physics: The Need for an EKF');
xlabel('Time (s)'); ylabel('Angle (Degrees)');
grid on;

%% 4. Export to C-Header File for Bare-Metal Simulation
header_file = fullfile('..', 'c_ekf_core', 'mems_sensor_data.h');
fid = fopen(header_file, 'w');

fprintf(fid, '/* Auto-generated MEMS Sensor Data for Embedded EKF */\n');
fprintf(fid, '#ifndef MEMS_SENSOR_DATA_H\n#define MEMS_SENSOR_DATA_H\n\n');
fprintf(fid, '#define DATA_LENGTH %d\n', N);
fprintf(fid, '#define DT %f\n\n', dt);

% Write Accelerometer Array
fprintf(fid, 'const float accel_meas[DATA_LENGTH] = {');
fprintf(fid, '%.4f, ', accel_meas(1:end-1));
fprintf(fid, '%.4f};\n\n', accel_meas(end));

% Write Gyroscope Array
fprintf(fid, 'const float gyro_meas[DATA_LENGTH] = {');
fprintf(fid, '%.4f, ', gyro_meas(1:end-1));
fprintf(fid, '%.4f};\n\n', gyro_meas(end));

% Write Ground Truth (for calculating EKF accuracy later)
fprintf(fid, 'const float true_angle[DATA_LENGTH] = {');
fprintf(fid, '%.4f, ', true_angle(1:end-1));
fprintf(fid, '%.4f};\n\n', true_angle(end));

fprintf(fid, '#endif // MEMS_SENSOR_DATA_H\n');
fclose(fid);

disp('Simulation complete. Physical graph generated.');
disp(['C Header file written to: ', header_file]);