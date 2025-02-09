% Parámetros de la balanza
L = 0.4; % Longitud del brazo (m)
m = 0.015; % Masa de la pelota (kg)
I = m * L^2; % Momento de inercia (kg·m^2)
b = 0.01; % Coeficiente de fricción (N·m·s/rad)
Kp = 10; % Ganancia proporcional del PID
Kd = 1; % Ganancia derivativa del PID
Ki = 0.5; % Ganancia integral del PID
torque_max = 11; % Torque máximo del motor (kg·cm)

% Condiciones iniciales
theta = 0.1; % Ángulo inicial (rad)
theta_dot = 0; % Velocidad angular inicial (rad/s)
theta_ref = 0; % Ángulo deseado (rad)
dt = 0.01; % Paso de tiempo (s)
t_sim = 5; % Tiempo de simulación (s)

% Variables para almacenar resultados
time = 0:dt:t_sim;
theta_history = zeros(size(time));
theta_dot_history = zeros(size(time));

% Inicialización de variables PID
integral = 0;
error_prev = 0;

% Simulación
for i = 1:length(time)
    % Error actual
    error = theta_ref - theta;
    integral = integral + error * dt;
    derivative = (error - error_prev) / dt;

    % Controlador PID
    u = Kp * error + Ki * integral + Kd * derivative;

    % Limitar el torque del motor
    u = max(min(u, torque_max), -torque_max);

    % Dinámica de la balanza
    theta_ddot = (u - b * theta_dot) / I;
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;

    % Almacenar resultados
    theta_history(i) = theta;
    theta_dot_history(i) = theta_dot;

    % Actualizar error previo
    error_prev = error;
end

% Gráficas de resultados
figure;
subplot(2,1,1);
plot(time, theta_history, 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Ángulo (rad)');
title('Respuesta del sistema');
grid on;

subplot(2,1,2);
plot(time, theta_dot_history, 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rad/s)');
grid on;