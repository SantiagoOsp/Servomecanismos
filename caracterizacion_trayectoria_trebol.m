%% CARACTERIZACIÓN, CINEMÁTICA Y DINÁMICA DE ROBOT 2R
clear; clc; close all;

%% 1. Parámetros del Proyecto 
L = 0.2;           % Lado
n = 7;              % Número de hojas
v_const = 0.1;      % Velocidad deseada
S = 1.25;           % Escala
beta = deg2rad(45); % Rotación

% Longitud de eslabones, masas y centros de gravedad
l1 = 0.195; m_link1 = 0.700; cg1 = l1 / 2; % cg1 es el centro de gravedad
l2 = 0.25; m_link2 = 0.600; cg2 = l2 / 2; % Igual con cg2
m_motor2 = 0.300; m_tip = 0.050;
g = 9.81;
kgfcm_to_Nm = 0.0980665;
Nm_to_kgfcm = 1 / kgfcm_to_Nm;

% Actuadores actuales:
% - Base: motor de 30 kgf·cm nominal (37 rpm sin carga, 30 rpm con carga)
% - Codo/transmisión: JGB37-3530-270K (19 kgf·cm nominal)
motor_base.torque_nominal_kgfcm = 30;
motor_base.torque_stall_kgfcm = 90;
motor_base.rpm_no_load = 37;
motor_base.rpm_rated = 30;

motor_codo.torque_nominal_kgfcm = 19;
motor_codo.torque_stall_kgfcm = 60;
motor_codo.rpm_no_load = 37;
motor_codo.rpm_rated = 28;

motor_base.torque_nominal_Nm = motor_base.torque_nominal_kgfcm * kgfcm_to_Nm;
motor_base.torque_stall_Nm = motor_base.torque_stall_kgfcm * kgfcm_to_Nm;
motor_base.omega_no_load = 2*pi*motor_base.rpm_no_load / 60;
motor_base.omega_rated = 2*pi*motor_base.rpm_rated / 60;

motor_codo.torque_nominal_Nm = motor_codo.torque_nominal_kgfcm * kgfcm_to_Nm;
motor_codo.torque_stall_Nm = motor_codo.torque_stall_kgfcm * kgfcm_to_Nm;
motor_codo.omega_no_load = 2*pi*motor_codo.rpm_no_load / 60;
motor_codo.omega_rated = 2*pi*motor_codo.rpm_rated / 60;

% Transmisión externa (salida de motorreductor -> articulación)
% En base se usa correa dentada; en codo acople directo.
trans_base.ratio = 1.0;   % >1 aumenta torque a costa de velocidad
trans_base.eff = 0.92;    % eficiencia de correa
trans_codo.ratio = 1.0;
trans_codo.eff = 0.95;

% Inercias del tren motriz vistas en el eje de salida del motorreductor (ajustables con CAD)
J_motor_out_base = 1.2e-5; % [kg m^2]
J_motor_out_codo = 1.2e-5; % [kg m^2]
J_polea_base = 4.0e-5;     % [kg m^2]
J_polea_codo = 1.5e-5;     % [kg m^2]

%% 2. Generación de la Trayectoria Base del Trébol
r0 = (L/2) * 0.75;  
A = (L/2) * 0.25;   

phi_raw = linspace(0, 2*pi, 1000); 
r_raw = r0 + A*cos(n * phi_raw);
x_raw = r_raw .* cos(phi_raw);
y_raw = r_raw .* sin(phi_raw);

% Parametrización para velocidad constante
dx = diff(x_raw); dy = diff(y_raw);
ds = sqrt(dx.^2 + dy.^2);
s_acumulada = [0, cumsum(ds)];
T_total = s_acumulada(end) / v_const;
dt = 0.01; % Paso de tiempo de simulación
t = 0:dt:T_total;

s_t = v_const * t; 
phi_t = interp1(s_acumulada, phi_raw, s_t, 'pchip', 'extrap');

r_t = r0 + A*cos(n * phi_t);
x_traj = S * r_t .* cos(phi_t + beta);
y_traj = S * r_t .* sin(phi_t + beta);
x_traj(end) = x_traj(1); y_traj(end) = y_traj(1);

% Desplazamiento del Origen al espacio de trabajo seguro
X_centro = 0.2;  Y_centro = 0.2; 
x_traj_desp = x_traj + X_centro;
y_traj_desp = y_traj + Y_centro;

%% 3. Posición "Home" (Replegado Seguro, evitando singularidad)
disp('1. Calculando posición Home (Segura)...');

% Puño a la izquierda (x<0) y por encima (y>0) del origen
th1_home = deg2rad(120);           
th2_home = deg2rad(-40);             

% Calculamos la posición inicial (x_inicio, y_inicio) por cinemática directa
x_inicio = l1 * cos(th1_home) + l2 * cos(th1_home + th2_home);
y_inicio = l1 * sin(th1_home) + l2 * sin(th1_home + th2_home);

%% 4. Punto de inicio tangencial (NUEVO)
disp('2. Buscando punto de entrada tangencial...');
% 1. Encontrar el vector tangente en cada punto de la trayectoria
dx_traj = gradient(x_traj_desp);
dy_traj = gradient(y_traj_desp);
norm_traj = sqrt(dx_traj.^2 + dy_traj.^2);
tx = dx_traj ./ norm_traj; 
ty = dy_traj ./ norm_traj;

% 2. Encontrar el vector desde Home hacia cada punto
hx = x_traj_desp - x_inicio;
hy = y_traj_desp - y_inicio;
norm_h = sqrt(hx.^2 + hy.^2);
hx = hx ./ norm_h;
hy = hy ./ norm_h;

% 3. Producto punto para forzar que sea paralela a la tangente
alineacion = hx .* tx + hy .* ty;
[~, idx_tangente] = max(alineacion); % El valor más cercano a 1

x_temp = x_traj_desp(1:end-1); y_temp = y_traj_desp(1:end-1);
x_traj_opt = [x_temp(idx_tangente:end), x_temp(1:idx_tangente-1)];
y_traj_opt = [y_temp(idx_tangente:end), y_temp(1:idx_tangente-1)];

x_traj_desp = [x_traj_opt, x_traj_opt(1)];
y_traj_desp = [y_traj_opt, y_traj_opt(1)];

%% 5. Fase de Aproximación (Curva de Bezier Fluida y Tangente)
disp('3. Generando curva de aproximación fluida...');
dist_aprox = sqrt((x_traj_desp(1) - x_inicio)^2 + (y_traj_desp(1) - y_inicio)^2);

% 1. Puntos de inicio y fin
P0x = x_inicio;       P0y = y_inicio;
P3x = x_traj_desp(1); P3y = y_traj_desp(1);

% 2. Vector tangente inicial del trébol para asegurar una entrada sin golpes
dx_llegada = x_traj_desp(2) - x_traj_desp(1);
dy_llegada = y_traj_desp(2) - y_traj_desp(1);
norm_llegada = sqrt(dx_llegada^2 + dy_llegada^2);
tx_in = dx_llegada / norm_llegada;
ty_in = dy_llegada / norm_llegada;

% 3. Puntos de control (P1 y P2) para una aproximación amplia y curva
L_ctrl = dist_aprox * 0.55; % Radio de curvatura más amplio

% Para llegar de forma 100% paralela a la trayectoria
P2x = P3x - L_ctrl * tx_in;
P2y = P3y - L_ctrl * ty_in;

% Hacemos que la salida rompa en un arco visible hacia un lado
% Generamos un vector perpendicular a la línea recta entre Home y el Trébol
vx = P3x - P0x; vy = P3y - P0y;
nx = -vy/norm([vx, vy]); ny = vx/norm([vx, vy]);

P1x = P0x + vx*0.3 + nx*L_ctrl; 
P1y = P0y + vy*0.3 + ny*L_ctrl;

% 4. Perfil de tiempo diseñado para un acople perfecto de velocidad y aceleración cero.
T_aprox = (3 * L_ctrl) / v_const;
t_aprox = 0:dt:T_aprox;

% t normalizado en el rango [0, 1]
x_norm = t_aprox / T_aprox;

% Vector de posición paramétrica para arranque/parada suaves.


tau = 10*x_norm.^3 - 15*x_norm.^4 + 6*x_norm.^5;

% Condiciones: tau(0)=0, tau'(0)=0, tau''(0)=0
% tau(1)=1, tau'(1) = v_const * T_aprox / (3*L_ctrl) = C=1, tau''(1)=0
% Polinomio: tau(x) = a*x^5 + b*x^4 + c*x^3
% Resolviendo el sistema (a=3, b=-8, c=6):
tau = 3*x_norm.^5 - 8*x_norm.^4 + 6*x_norm.^3;

% 5. Polinomio de Interpolación Cúbica de Bezier
x_aprox = (1-tau).^3 * P0x + 3*(1-tau).^2.*tau * P1x + 3*(1-tau).*tau.^2 * P2x + tau.^3 * P3x;
y_aprox = (1-tau).^3 * P0y + 3*(1-tau).^2.*tau * P1y + 3*(1-tau).*tau.^2 * P2y + tau.^3 * P3y;
x_aprox(end) = []; y_aprox(end) = [];

x_total = [x_aprox, x_traj_desp];
y_total = [y_aprox, y_traj_desp];
num_puntos = length(x_total);

%% 6. Cinemática Inversa Vectorizada (Codo Arriba)
disp('4. Calculando Cinemática Inversa...');
theta1_raw = zeros(1, num_puntos);
theta2_raw = zeros(1, num_puntos);

for i = 1:num_puntos
    if i == 1
        % Forzar explícitamente los ángulos del primer punto (Home)
        % para evitar el salto de signo de atan2(0,-1) que resulta en pi en vez de -pi
        th1 = th1_home;
        th2 = th2_home;
    else
        x = x_total(i); y = y_total(i);
        D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
        if D > 1, D = 1; elseif D < -1, D = -1; end
        
        th2 = atan2(-sqrt(1 - D^2), D);  
        th1 = atan2(y, x) - atan2(l2 * sin(th2), l1 + l2 * cos(th2));
    end
    
    theta1_raw(i) = th1; theta2_raw(i) = th2;
end
theta1_raw = unwrap(theta1_raw);
theta2_raw = unwrap(theta2_raw);

%% 6.1 Cálculo del rango de movimiento requerido por articulación
rango_th1_deg = rad2deg(max(theta1_raw) - min(theta1_raw));
rango_th2_deg = rad2deg(max(theta2_raw) - min(theta2_raw));

fprintf('\n==== REQUISITOS DE MOVIMIENTO (RANGO TOTAL) ====\n');
fprintf('Eslabón 1 (Theta 1): Rango de %.2f grados [Mín: %.2f°, Máx: %.2f°]\n', rango_th1_deg, rad2deg(min(theta1_raw)), rad2deg(max(theta1_raw)));
fprintf('Eslabón 2 (Theta 2): Rango de %.2f grados [Mín: %.2f°, Máx: %.2f°]\n', rango_th2_deg, rad2deg(min(theta2_raw)), rad2deg(max(theta2_raw)));
fprintf('================================================\n\n');

%% 7. Filtrado SavGol (Básicamente no se usa ahora, pero será necesario para procesar las respuestas del microcontrolador)
disp('5. Optimizando perfiles para motores (Filtrado Savitzky-Golay)...');
orden_pol = 3;       
window_size = 5;    

theta1 = sgolayfilt(theta1_raw, orden_pol, window_size);
theta2 = sgolayfilt(theta2_raw, orden_pol, window_size);

dtheta1 = sgolayfilt(gradient(theta1, dt), orden_pol, window_size);
dtheta2 = sgolayfilt(gradient(theta2, dt), orden_pol, window_size);

ddtheta1 = sgolayfilt(gradient(dtheta1, dt), orden_pol, window_size);
ddtheta2 = sgolayfilt(gradient(dtheta2, dt), orden_pol, window_size);

t_total_vec = (0:num_puntos-1) * dt;

%% 8. Que tanto se aleja el filtrado de el ideal?
x_ideal = l1 * cos(theta1_raw) + l2 * cos(theta1_raw + theta2_raw);
y_ideal = l1 * sin(theta1_raw) + l2 * sin(theta1_raw + theta2_raw);

x_real = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
y_real = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

figure(1); clf; hold on; axis equal; grid on;

% Calculamos y dibujamos el cuadrado delimitador en base a L y a la escala S
L_scaled = L * S;
sq_x = X_centro + [-1, 1, 1, -1, -1] * (L_scaled / 2);
sq_y = Y_centro + [-1, -1, 1, 1, -1] * (L_scaled / 2);
plot(sq_x, sq_y, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Cuadrado Delimitador');

plot(x_ideal, y_ideal, 'Color', [0.8 0.8 0.8], 'LineWidth', 3, 'DisplayName', 'Trébol Matemático Ideal');
plot(x_real, y_real, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trébol Suavizado (Motores)');
title('Comparación: Trayectoria Ideal vs Suavizada');
legend('Location', 'best');

%% 9. Dinámica Inversa y Cálculo de Torques (ACTUALIZADA CON cg1 y cg2)
disp('6. Calculando Dinámica Inversa...');
m1_t = m_link1 + m_motor2;
lc1_t = (m_link1 * cg1 + m_motor2 * l1) / m1_t; 
% NOTA: Si tienes el Izz de CAD, reemplaza el término (1/12)*m*l^2 por ese Izz
I1_t = (1/12)*m_link1*l1^2 + m_link1*(lc1_t - cg1)^2 + m_motor2*(l1 - lc1_t)^2; 

m2_t = m_link2 + m_tip;
lc2_t = (m_link2 * cg2 + m_tip * l2) / m2_t;
I2_t = (1/12)*m_link2*l2^2 + m_link2*(lc2_t - cg2)^2 + m_tip*(l2 - lc2_t)^2;

tau1 = zeros(1, num_puntos); tau2 = zeros(1, num_puntos);
tau1_inercial = zeros(1, num_puntos); tau2_inercial = zeros(1, num_puntos);
tau1_no_inercial = zeros(1, num_puntos); tau2_no_inercial = zeros(1, num_puntos);
Jeq1_joint = zeros(1, num_puntos); Jeq2_joint = zeros(1, num_puntos);
maniobrabilidad = zeros(1, num_puntos);
cond_J = zeros(1, num_puntos);

for i = 1:num_puntos
    th1 = theta1(i); th2 = theta2(i);
    d1 = dtheta1(i); d2 = dtheta2(i);
    dd1 = ddtheta1(i); dd2 = ddtheta2(i);
    
    M11 = m1_t*lc1_t^2 + m2_t*(l1^2 + lc2_t^2 + 2*l1*lc2_t*cos(th2)) + I1_t + I2_t;
    M12 = m2_t*(lc2_t^2 + l1*lc2_t*cos(th2)) + I2_t;
    M22 = m2_t*lc2_t^2 + I2_t;
    
    h_coriolis = -m2_t*l1*lc2_t*sin(th2);
    C1 = h_coriolis*(2*d1*d2 + d2^2);
    C2 = -h_coriolis*d1^2;
    
    G1_dyn = (m1_t*lc1_t + m2_t*l1)*g*cos(th1) + m2_t*lc2_t*g*cos(th1 + th2);
    G2_dyn = m2_t*lc2_t*g*cos(th1 + th2);
    
    tau1_inercial(i) = M11*dd1 + M12*dd2;
    tau2_inercial(i) = M12*dd1 + M22*dd2;
    tau1_no_inercial(i) = C1 + G1_dyn;
    tau2_no_inercial(i) = C2 + G2_dyn;
    
    tau1(i) = tau1_inercial(i) + tau1_no_inercial(i);
    tau2(i) = tau2_inercial(i) + tau2_no_inercial(i);
    
    Jeq1_joint(i) = M11;
    Jeq2_joint(i) = M22;

    % Índices de maniobrabilidad (Yoshikawa + condición del Jacobiano)
    J11 = -l1*sin(th1) - l2*sin(th1 + th2);
    J12 = -l2*sin(th1 + th2);
    J21 =  l1*cos(th1) + l2*cos(th1 + th2);
    J22 =  l2*cos(th1 + th2);
    Jmat = [J11, J12; J21, J22];
    maniobrabilidad(i) = sqrt(max(det(Jmat*Jmat'), 0));
    cond_J(i) = cond(Jmat);
end

% Magnitudes de articulación y de motor
tau1_kgfcm = tau1 * Nm_to_kgfcm;
tau2_kgfcm = tau2 * Nm_to_kgfcm;

omega_m_base = dtheta1 * trans_base.ratio;
omega_m_codo = dtheta2 * trans_codo.ratio;
alpha_m_base = ddtheta1 * trans_base.ratio;
alpha_m_codo = ddtheta2 * trans_codo.ratio;

tau_m_base_req = tau1 ./ (trans_base.ratio * trans_base.eff);
tau_m_codo_req = tau2 ./ (trans_codo.ratio * trans_codo.eff);

Jref_base = Jeq1_joint ./ (trans_base.ratio^2) + J_motor_out_base + J_polea_base;
Jref_codo = Jeq2_joint ./ (trans_codo.ratio^2) + J_motor_out_codo + J_polea_codo;

tau_m_base_inercia = Jref_base .* alpha_m_base;
tau_m_codo_inercia = Jref_codo .* alpha_m_codo;

% Porcentajes de carga (torque, velocidad y punto operativo velocidad-torque)
pct_torque_nom_base = 100 * abs(tau_m_base_req) / motor_base.torque_nominal_Nm;
pct_torque_nom_codo = 100 * abs(tau_m_codo_req) / motor_codo.torque_nominal_Nm;
pct_torque_stall_base = 100 * abs(tau_m_base_req) / motor_base.torque_stall_Nm;
pct_torque_stall_codo = 100 * abs(tau_m_codo_req) / motor_codo.torque_stall_Nm;
pct_speed_base = 100 * abs(omega_m_base) / motor_base.omega_no_load;
pct_speed_codo = 100 * abs(omega_m_codo) / motor_codo.omega_no_load;

tau_disp_base = max(0, motor_base.torque_stall_Nm * (1 - abs(omega_m_base)/motor_base.omega_no_load));
tau_disp_codo = max(0, motor_codo.torque_stall_Nm * (1 - abs(omega_m_codo)/motor_codo.omega_no_load));
pct_carga_oper_base = 100 * abs(tau_m_base_req) ./ max(tau_disp_base, 1e-9);
pct_carga_oper_codo = 100 * abs(tau_m_codo_req) ./ max(tau_disp_codo, 1e-9);
margen_tau_base = tau_disp_base - abs(tau_m_base_req);
margen_tau_codo = tau_disp_codo - abs(tau_m_codo_req);

maniobrabilidad_pct = 100 * maniobrabilidad / (l1*l2);

fprintf('\n==== CARGA DE MOTORES Y MANIOBRABILIDAD ====\n');
fprintf('Base: pico torque req = %.2f kgf·cm (%.1f%% nominal, %.1f%% stall)\n', max(abs(tau_m_base_req))*Nm_to_kgfcm, max(pct_torque_nom_base), max(pct_torque_stall_base));
fprintf('Codo: pico torque req = %.2f kgf·cm (%.1f%% nominal, %.1f%% stall)\n', max(abs(tau_m_codo_req))*Nm_to_kgfcm, max(pct_torque_nom_codo), max(pct_torque_stall_codo));
fprintf('Base: carga operativa máxima velocidad-torque = %.1f%%, margen mínimo = %.2f kgf·cm\n', max(pct_carga_oper_base), min(margen_tau_base)*Nm_to_kgfcm);
fprintf('Codo: carga operativa máxima velocidad-torque = %.1f%%, margen mínimo = %.2f kgf·cm\n', max(pct_carga_oper_codo), min(margen_tau_codo)*Nm_to_kgfcm);
fprintf('Base: inercia reflejada promedio = %.6f kg·m²\n', mean(Jref_base));
fprintf('Codo: inercia reflejada promedio = %.6f kg·m²\n', mean(Jref_codo));
fprintf('Maniobrabilidad: min %.1f%%, media %.1f%% (índice cond(J) máx = %.2f)\n', min(maniobrabilidad_pct), mean(maniobrabilidad_pct), max(cond_J));
fprintf('=============================================\n\n');


%% 10. Gráficas Cinemáticas y de Torque
figure(2); clf;
subplot(4,1,1); plot(t_total_vec, rad2deg(theta1), 'b', t_total_vec, rad2deg(theta2), 'r'); title('Posiciones Articulares (grados)'); legend('\theta_1', '\theta_2', 'Location', 'best'); grid on;
subplot(4,1,2); plot(t_total_vec, dtheta1, 'b', t_total_vec, dtheta2, 'r'); title('Velocidades (rad/s)'); legend('\omega_1', '\omega_2', 'Location', 'best'); grid on;
subplot(4,1,3); plot(t_total_vec, ddtheta1, 'b', t_total_vec, ddtheta2, 'r'); title('Aceleraciones Suavizadas (rad/s^2)'); legend('\alpha_1', '\alpha_2', 'Location', 'best'); grid on;
subplot(4,1,4); plot(t_total_vec, tau1_kgfcm, 'b', t_total_vec, tau2_kgfcm, 'r'); title('Torques (kgf·cm)'); xlabel('Tiempo [s]'); legend('\tau_1', '\tau_2', 'Location', 'best'); grid on;

figure(4); clf;
subplot(3,1,1);
plot(t_total_vec, pct_torque_nom_base, 'b', t_total_vec, pct_torque_nom_codo, 'r', 'LineWidth', 1.2); hold on;
yline(100, 'k--', 'Nominal');
yline(100*motor_base.torque_stall_Nm/motor_base.torque_nominal_Nm, 'k:', 'Stall');
title('Carga por torque respecto al nominal (%)');
legend('Base', 'Codo', 'Location', 'best'); grid on;

subplot(3,1,2);
plot(t_total_vec, pct_carga_oper_base, 'b', t_total_vec, pct_carga_oper_codo, 'r', 'LineWidth', 1.2); hold on;
yline(100, 'k--', 'Límite operativo');
title('Carga en curva velocidad-torque (%)');
legend('Base', 'Codo', 'Location', 'best'); grid on;

subplot(3,1,3);
yyaxis left
plot(t_total_vec, Jref_base, 'b', t_total_vec, Jref_codo, 'r', 'LineWidth', 1.2);
ylabel('Inercia reflejada [kg·m^2]');
yyaxis right
plot(t_total_vec, maniobrabilidad_pct, 'k', 'LineWidth', 1.2);
ylabel('Maniobrabilidad [%]');
xlabel('Tiempo [s]');
title('Inercia reflejada y maniobrabilidad');
legend('J_{ref} Base', 'J_{ref} Codo', 'Maniobrabilidad', 'Location', 'best');
grid on;

%% 11. Animación del Robot
figure(3); clf; hold on; axis equal; grid on; axis([-0.25 0.6 -0.05 0.6]);
plot([-0.25, 0.6], [0, 0], 'k-', 'LineWidth', 3); % Mesa

% Dibujamos también dentro de la animación el cuadrado
plot(sq_x, sq_y, 'g--', 'LineWidth', 1.5); % Límite cuadrado del trébol en verde

plot(x_total, y_total, 'color', [0.8 0.8 0.8], 'LineWidth', 1.5); % Ruta

h_robot = plot([0, l1*cos(theta1(1)), x_real(1)], [0, l1*sin(theta1(1)), y_real(1)], '-o', 'LineWidth', 4, 'Color', '#D95319', 'MarkerFaceColor', '#0072BD');
h_rastro = plot(x_real(1), y_real(1), 'b-', 'LineWidth', 2);

x_rastro = zeros(1, num_puntos); y_rastro = zeros(1, num_puntos);
paso = 15;

disp('7. Reproduciendo animación...');
for i = 1:paso:num_puntos
    x_c = l1*cos(theta1(i)); y_c = l1*sin(theta1(i));
    x_rastro(1:i) = x_real(1:i); y_rastro(1:i) = y_real(1:i);
    
    set(h_robot, 'XData', [0, x_c, x_real(i)], 'YData', [0, y_c, y_real(i)]);
    set(h_rastro, 'XData', x_rastro(1:i), 'YData', y_rastro(1:i));
    drawnow;
end
set(h_robot, 'XData', [0, l1*cos(theta1(end)), x_real(end)], 'YData', [0, l1*sin(theta1(end)), y_real(end)]);
set(h_rastro, 'XData', x_real, 'YData', y_real); drawnow;
disp('Proceso finalizado.');
