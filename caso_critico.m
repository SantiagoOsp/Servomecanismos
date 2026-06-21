
%% 12. BARRIDO DE PARÁMETROS: BÚSQUEDA DEL CASO CRÍTICO (WORST-CASE SCENARIO)
disp('Iniciando análisis del Caso Crítico. Esto puede tardar unos segundos...');

% 1. Definir los vectores de prueba (resolución ajustable)
vec_S = linspace(0.75, 1.25, 5);        % 10 escalas
vec_n = 4:7;                             % 4 tipos de trébol
vec_v = linspace(0.01, 0.1, 10);         % 10 velocidades
vec_beta = deg2rad(linspace(-45, 45, 45)); % 180 rotaciones

% 2. Variables para rastrear los máximos históricos
max_global_tau1 = 0; max_global_tau2 = 0;
max_global_acc1 = 0; max_global_acc2 = 0;
max_global_vel1 = 0; max_global_vel2 = 0; % NUEVO: Rastreadores de velocidad máxima
params_criticos = struct('S', 0, 'n', 0, 'v', 0, 'beta', 0);

for S_i = vec_S
    for n_i = vec_n
        for v_i = vec_v
            for beta_i = vec_beta
                
                % --- A. Generación de Trayectoria Rápida ---
                phi_raw = linspace(0, 2*pi, 500); 
                r_raw = r0 + A*cos(n_i * phi_raw);
                x_raw = r_raw .* cos(phi_raw);
                y_raw = r_raw .* sin(phi_raw);
                
                dx = diff(x_raw); dy = diff(y_raw);
                ds = sqrt(dx.^2 + dy.^2);
                s_acum = [0, cumsum(ds)];
                
                T_tot = s_acum(end) / v_i;
                dt_sim = 0.01; 
                t_sim = 0:dt_sim:T_tot;
                s_t = v_i * t_sim; 
                
                phi_t = interp1(s_acum, phi_raw, s_t, 'pchip', 'extrap');
                r_t = r0 + A*cos(n_i * phi_t);
                
                x_traj = S_i * r_t .* cos(phi_t + beta_i) + X_centro;
                y_traj = S_i * r_t .* sin(phi_t + beta_i) + Y_centro;
                
                % --- A.2 Home, Ajuste de Tangente y Aproximación Bezier ---
                th1_home = pi/2;            
                th2_home = -pi;             
                x_inicio = l1 * cos(th1_home) + l2 * cos(th1_home + th2_home);
                y_inicio = l1 * sin(th1_home) + l2 * sin(th1_home + th2_home);

                dx_traj = gradient(x_traj); dy_traj = gradient(y_traj);
                norm_traj = sqrt(dx_traj.^2 + dy_traj.^2);
                tx = dx_traj ./ norm_traj; ty = dy_traj ./ norm_traj;
                
                hx = x_traj - x_inicio; hy = y_traj - y_inicio;
                norm_h = sqrt(hx.^2 + hy.^2); 
                hx = hx ./ norm_h; hy = hy ./ norm_h;
                
                alineacion = hx .* tx + hy .* ty;
                [~, idx_tangente] = max(alineacion); 
                
                x_temp = x_traj(1:end-1); y_temp = y_traj(1:end-1);
                x_traj_opt = [x_temp(idx_tangente:end), x_temp(1:idx_tangente-1)];
                y_traj_opt = [y_temp(idx_tangente:end), y_temp(1:idx_tangente-1)];
                
                x_traj_desp = [x_traj_opt, x_traj_opt(1)];
                y_traj_desp = [y_traj_opt, y_traj_opt(1)];

                dist_aprox = sqrt((x_traj_desp(1) - x_inicio)^2 + (y_traj_desp(1) - y_inicio)^2);
                P0x = x_inicio;       P0y = y_inicio;
                P3x = x_traj_desp(1); P3y = y_traj_desp(1);
                
                dx_llegada = x_traj_desp(2) - x_traj_desp(1);
                dy_llegada = y_traj_desp(2) - y_traj_desp(1);
                norm_llegada = sqrt(dx_llegada^2 + dy_llegada^2);
                tx_in = dx_llegada / norm_llegada; ty_in = dy_llegada / norm_llegada;
                
                L_ctrl = dist_aprox * 0.4;
                P2x = P3x - L_ctrl * tx_in; P2y = P3y - L_ctrl * ty_in;
                P1x = P0x + L_ctrl * 1.5;   P1y = P0y + L_ctrl * 0.5;
                
                T_aprox = (6 * L_ctrl) / v_i; 
                t_aprox = 0:dt_sim:T_aprox;
                tau = (t_aprox / T_aprox).^2; 
                
                x_aprox = (1-tau).^3 * P0x + 3*(1-tau).^2.*tau * P1x + 3*(1-tau).*tau.^2 * P2x + tau.^3 * P3x;
                y_aprox = (1-tau).^3 * P0y + 3*(1-tau).^2.*tau * P1y + 3*(1-tau).*tau.^2 * P2y + tau.^3 * P3y;
                x_aprox(end) = []; y_aprox(end) = [];
                
                x_total_iter = [x_aprox, x_traj_desp];
                y_total_iter = [y_aprox, y_traj_desp];
                
                % --- B. Comprobación de Espacio de Trabajo ---
                D_test = (x_total_iter.^2 + y_total_iter.^2 - l1^2 - l2^2) / (2 * l1 * l2);
                if any(D_test > 1) || any(D_test < -1)
                    continue; 
                end
                
                % --- C. Cinemática Inversa ---
                num_p_iter = length(x_total_iter);
                th1_iter = zeros(1, num_p_iter); th2_iter = zeros(1, num_p_iter);
                for i_k = 1:num_p_iter
                    if i_k == 1
                        th1_iter(i_k) = th1_home; th2_iter(i_k) = th2_home;
                    else
                        th2_iter(i_k) = atan2(-sqrt(1 - D_test(i_k)^2), D_test(i_k));  
                        th1_iter(i_k) = atan2(y_total_iter(i_k), x_total_iter(i_k)) - atan2(l2 * sin(th2_iter(i_k)), l1 + l2 * cos(th2_iter(i_k)));
                    end
                end
                th1_iter = unwrap(th1_iter); th2_iter = unwrap(th2_iter);
                
                % --- D. Filtrado de Velocidad y Aceleración ---
                th1_filt = sgolayfilt(th1_iter, 3, 21);
                th2_filt = sgolayfilt(th2_iter, 3, 21);
                
                d1_iter = sgolayfilt(gradient(th1_filt, dt_sim), 3, 21);
                d2_iter = sgolayfilt(gradient(th2_filt, dt_sim), 3, 21);
                
                dd1_iter = sgolayfilt(gradient(d1_iter, dt_sim), 3, 21);
                dd2_iter = sgolayfilt(gradient(d2_iter, dt_sim), 3, 21);
                
                % --- E. Dinámica Inversa Rápida ---
                M11 = m1_t*lc1_t^2 + m2_t*(l1^2 + lc2_t^2 + 2*l1*lc2_t*cos(th2_filt)) + I1_t + I2_t;
                M12 = m2_t*(lc2_t^2 + l1*lc2_t*cos(th2_filt)) + I2_t;
                M22 = m2_t*lc2_t^2 + I2_t;
                
                h_cor = -m2_t*l1*lc2_t*sin(th2_filt);
                C1 = h_cor.*(2.*d1_iter.*d2_iter + d2_iter.^2);
                C2 = -h_cor.*d1_iter.^2;
                
                G1_dyn = (m1_t*lc1_t + m2_t*l1)*g*cos(th1_filt) + m2_t*lc2_t*g*cos(th1_filt + th2_filt);
                G2_dyn = m2_t*lc2_t*g*cos(th1_filt + th2_filt);
                
                tau1_iter = M11.*dd1_iter + M12.*dd2_iter + C1 + G1_dyn;
                tau2_iter = M12.*dd1_iter + M22.*dd2_iter + C2 + G2_dyn;
                
                tau1_kgfcm_iter = tau1_iter * (100 / 9.81);
                tau2_kgfcm_iter = tau2_iter * (100 / 9.81);
                
                % --- F. Evaluar y Actualizar Máximos ---
                max_t1_actual = max(abs(tau1_kgfcm_iter));
                max_t2_actual = max(abs(tau2_kgfcm_iter));
                max_v1_actual = max(abs(d1_iter)); % Máxima velocidad actual M1
                max_v2_actual = max(abs(d2_iter)); % Máxima velocidad actual M2
                
                % Actualizar picos absolutos de velocidad de forma independiente
                max_global_vel1 = max(max_global_vel1, max_v1_actual);
                max_global_vel2 = max(max_global_vel2, max_v2_actual);
                
                % Actualizar configuración crítica basada en el torque
                if max_t1_actual > max_global_tau1 || max_t2_actual > max_global_tau2
                    max_global_tau1 = max(max_global_tau1, max_t1_actual);
                    max_global_tau2 = max(max_global_tau2, max_t2_actual);
                    max_global_acc1 = max(max_global_acc1, max(abs(dd1_iter)));
                    max_global_acc2 = max(max_global_acc2, max(abs(dd2_iter)));
                    
                    params_criticos.S = S_i;
                    params_criticos.n = n_i;
                    params_criticos.v = v_i;
                    params_criticos.beta = rad2deg(beta_i);
                end
            end
        end
    end
end

%% 13. REPORTE DEL CASO CRÍTICO Y REQUERIMIENTOS
% Conversión de rad/s a RPM
rpm_m1 = max_global_vel1 * (60 / (2*pi));
rpm_m2 = max_global_vel2 * (60 / (2*pi));

fprintf('\n=======================================================\n');
fprintf('   REPORTE DE DIMENSIONAMIENTO: CASO CRÍTICO ABSOLUTO\n');
fprintf('=======================================================\n');
fprintf('Configuración que genera el mayor esfuerzo:\n');
fprintf('  - Escala (S): %.2f\n', params_criticos.S);
fprintf('  - Número de Hojas (n): %d\n', params_criticos.n);
fprintf('  - Velocidad (v): %.3f m/s\n', params_criticos.v);
fprintf('  - Rotación Inicial: %.1f°\n\n', params_criticos.beta);

fprintf('ESPECIFICACIONES MÍNIMAS PARA COMPRA DE MOTORES:\n');
fprintf('*(Incluye picos absolutos de todo el barrido de pruebas (Aproximación + Trayectoria))*\n\n');

fprintf('  MOTOR 1 (BASE):\n');
fprintf('    Velocidad Pico:   %.2f RPM  (%.2f rad/s)\n', rpm_m1, max_global_vel1);
fprintf('    Aceleración Pico: %.2f rad/s^2\n', max_global_acc1);
fprintf('    Torque Pico:      %.2f kgf·cm\n\n', max_global_tau1);

fprintf('  MOTOR 2 (CODO):\n');
fprintf('    Velocidad Pico:   %.2f RPM  (%.2f rad/s)\n', rpm_m2, max_global_vel2);
fprintf('    Aceleración Pico: %.2f rad/s^2\n', max_global_acc2);
fprintf('    Torque Pico:      %.2f kgf·cm\n', max_global_tau2);
fprintf('=======================================================\n\n');
disp('Nota 1: El motor seleccionado debe ser capaz de entregar la "Velocidad Pico" y el "Torque Pico" simultáneamente sin perder pasos o estancarse.');
disp('Nota 2: Aplica un margen de seguridad mecánico del 25% a estos valores al revisar los datasheets.');