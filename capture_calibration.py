#!/usr/bin/env python3
"""
CAPTURADOR DE CALIBRACIÓN - ROBOT 2R
======================================
⚠ Este script ha sido migrado a la aplicación PC completa.

Use en su lugar:
    cd Application/PC
    python main.py --cli

O para la GUI:
    cd Application/PC
    python main.py

La nueva aplicación incluye:
  - Captura de datos de calibración
  - Análisis y ajuste de modelos (K, τ, Ke, Kt)
  - Generación de trayectorias
  - Monitoreo en tiempo real
  - Interfaz gráfica completa
"""
print("⚠ Este script ha sido migrado a Application/PC/")
print("Ejecute: cd Application/PC && python main.py")
print()

import sys
sys.exit(0)
        """Conecta al puerto serial."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"✓ Conectado a {self.port} a {self.baud} baud")
            # Esperar señal READY
            t0 = time.time()
            while time.time() - t0 < 5:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if self.RE_READY.match(line):
                    print("✓ ESP32 lista")
                    return True
                if line:
                    print(f"  {line}")
            print("⚠ No se recibió READY, pero continuando...")
            return True
        except serial.SerialException as e:
            print(f"✗ Error conectando: {e}")
            return False

    def disconnect(self):
        """Desconecta."""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Desconectado")

    def send_command(self, cmd):
        """Envía un comando a la ESP32."""
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode())
            print(f"> {cmd}")
            return True
        return False

    def start_capture(self):
        """Inicia el hilo de captura."""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        """Bucle principal de lectura serial."""
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                self._process_line(line)
            except serial.SerialException:
                print("✗ Error de lectura serial")
                self.running = False
                break

    def _process_line(self, line):
        """Procesa una línea de texto del serial."""
        # Comandos
        m = self.RE_CMD.match(line)
        if m:
            self._process_command(m.group(1))
            return

        # Datos
        m = self.RE_DATA.match(line)
        if m:
            self._process_data(m.group(1))
            return

        # Resultados
        m = self.RE_RES.match(line)
        if m:
            self._process_result(m.group(1))
            return

        # Status
        m = self.RE_STATUS.match(line)
        if m:
            print(f"  STATUS: {m.group(1)}")
            return

        # Otros
        if line and line != 'READY':
            print(f"  {line}")

    def _process_command(self, cmd_str):
        """Procesa un mensaje de comando."""
        parts = cmd_str.split(':')
        cmd = parts[0]

        if cmd == 'SWEEP_START':
            motor = parts[1]
            print(f"\n{'='*60}")
            print(f"INICIANDO BARRIDO PWM - {motor}")
            print(f"{'='*60}")
            self.sweep_data[motor] = {'points': [], 'raw': []}

        elif cmd == 'SWEEP_POINT':
            motor = parts[1]
            pwm = float(parts[2])
            print(f"  PWM = {pwm*100:.1f}% ({pwm*255:.0f}/255)")

        elif cmd == 'SWEEP_RESULTS':
            motor = parts[1]
            print(f"  Recibiendo resultados...")

        elif cmd == 'SWEEP_END':
            motor = parts[1]
            print(f"✓ Barrido PWM {motor} completado")

        elif cmd == 'STEP_START':
            motor = parts[1]
            pwm = float(parts[2])
            dur = float(parts[3])
            print(f"\n{'='*60}")
            print(f"RESPUESTA AL ESCALÓN - {motor} (PWM={pwm*100:.1f}%, {dur:.1f}s)")
            print(f"{'='*60}")
            self.step_data[motor] = {'pwm': pwm, 'samples': []}

        elif cmd == 'STEP_DATA':
            motor = parts[1]
            n = int(parts[2])
            print(f"  Recibiendo {n} muestras...")

        elif cmd == 'STEP_END':
            motor = parts[1]
            n = len(self.step_data.get(motor, {}).get('samples', []))
            print(f"✓ Escalón {motor} completado ({n} muestras)")

        elif cmd == 'CURRENT_START':
            motor = parts[1]
            print(f"\n{'='*60}")
            print(f"PERFIL DE CORRIENTE - {motor}")
            print(f"{'='*60}")
            self.current_data[motor] = {'fwd': [], 'rev': []}

        elif cmd == 'CURRENT_END':
            motor = parts[1]
            print(f"✓ Perfil corriente {motor} completado")

        elif cmd == 'FULL_CALIB_START':
            print(f"\n{'='*60}")
            print("CALIBRACIÓN COMPLETA INICIADA")
            print(f"{'='*60}")

        elif cmd == 'FULL_CALIB_END':
            print(f"\n{'='*60}")
            print("CALIBRACIÓN COMPLETA FINALIZADA")
            print(f"{'='*60}")
            self._save_all_data()

        elif cmd == 'STOPPED':
            print("  ✓ Motores detenidos")

        elif cmd == 'BUSY':
            print("  ⚠ ESP32 ocupada, espere...")

        elif cmd == 'UNKNOWN':
            print(f"  ⚠ Comando desconocido: {parts[1]}")

    def _process_data(self, data_str):
        """Procesa una línea de datos."""
        parts = data_str.split(':')
        data_type = parts[0]

        if data_type == 'STEP':
            # DATA:STEP:<motor>:<t_us>:<pos>:<vel>:<accel>:<jerk>:<current>:<pwm>
            motor = parts[1]
            t_us = int(parts[2])
            pos = float(parts[3])
            vel = float(parts[4])
            accel = float(parts[5])
            jerk = float(parts[6])
            current = float(parts[7])
            pwm = int(parts[8])
            
            if motor in self.step_data:
                self.step_data[motor]['samples'].append({
                    't': t_us / 1e6,
                    'pos': pos,
                    'vel': vel,
                    'accel': accel,
                    'jerk': jerk,
                    'current': current,
                    'pwm': pwm
                })

        elif data_type == 'STEP_FULL':
            # DATA:STEP_FULL:<motor>:<t_us>:<pos>:<vel>:<current>:<pwm>
            motor = parts[1]
            t_us = int(parts[2])
            pos = float(parts[3])
            vel = float(parts[4])
            current = float(parts[5])
            pwm = int(parts[6])
            
            if motor in self.step_data:
                self.step_data[motor]['samples'].append({
                    't': t_us / 1e6,
                    'pos': pos,
                    'vel': vel,
                    'current': current,
                    'pwm': pwm
                })

        elif data_type in ('BASE', 'CODO'):
            # DATA:<motor>:<t_us>:<pos>:<vel>:<current>:<pwm>
            motor = parts[0]
            t_us = int(parts[1])
            pos = float(parts[2])
            vel = float(parts[3])
            current = float(parts[4])
            pwm = int(parts[5])
            
            if motor in self.sweep_data:
                self.sweep_data[motor]['raw'].append({
                    't': t_us / 1e6,
                    'pos': pos,
                    'vel': vel,
                    'current': current,
                    'pwm': pwm
                })

        elif data_type == 'CURRENT':
            # DATA:CURRENT:<motor>:<pwm>:<vel>:<curr_R>:<curr_L>
            # DATA:CURRENT:REV_<motor>:<pwm>:<vel>:<curr_R>:<curr_L>
            motor_full = parts[1]
            pwm = float(parts[2])
            vel = float(parts[3])
            curr_r = float(parts[4])
            curr_l = float(parts[5])
            
            is_rev = motor_full.startswith('REV_')
            motor = motor_full.replace('REV_', '') if is_rev else motor_full
            key = 'rev' if is_rev else 'fwd'
            
            if motor in self.current_data:
                self.current_data[motor][key].append({
                    'pwm': pwm,
                    'vel': vel,
                    'curr_R': curr_r,
                    'curr_L': curr_l
                })

    def _process_result(self, res_str):
        """Procesa una línea de resultados."""
        parts = res_str.split(':')
        pwm = float(parts[0])
        vel = float(parts[1])
        vel_std = float(parts[2])
        curr = float(parts[3])
        
        print(f"    PWM={pwm*100:.1f}% → vel={vel:.3f} rad/s, corriente={curr:.3f} A")

    def _save_all_data(self):
        """Guarda todos los datos recolectados en CSV y genera gráficas."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # --- Guardar barridos PWM ---
        for motor, data in self.sweep_data.items():
            if not data['raw']:
                continue
            self._save_sweep_csv(motor, data, timestamp)
        
        # --- Guardar respuestas al escalón ---
        for motor, data in self.step_data.items():
            if not data['samples']:
                continue
            self._save_step_csv(motor, data, timestamp)
        
        # --- Guardar perfiles de corriente ---
        for motor, data in self.current_data.items():
            if not data['fwd'] and not data['rev']:
                continue
            self._save_current_csv(motor, data, timestamp)
        
        # --- Generar gráficas ---
        if plt:
            self._generate_plots(timestamp)
        
        print(f"\n✓ Datos guardados en {self.output_dir}/{timestamp}/")

    def _save_sweep_csv(self, motor, data, timestamp):
        """Guarda datos de barrido PWM."""
        dir_path = os.path.join(self.output_dir, timestamp)
        os.makedirs(dir_path, exist_ok=True)
        
        path = os.path.join(dir_path, f"sweep_{motor}.csv")
        with open(path, 'w') as f:
            f.write("t_s,pos_rad,vel_rads,current_A,pwm_raw\n")
            for s in data['raw']:
                f.write(f"{s['t']},{s['pos']:.8f},{s['vel']:.8f},{s['current']:.6f},{s['pwm']}\n")
        print(f"  → {path}")

    def _save_step_csv(self, motor, data, timestamp):
        """Guarda datos de respuesta al escalón."""
        dir_path = os.path.join(self.output_dir, timestamp)
        os.makedirs(dir_path, exist_ok=True)
        
        path = os.path.join(dir_path, f"step_{motor}_pwm{data['pwm']:.0%}.csv")
        with open(path, 'w') as f:
            # Detectar campos disponibles
            sample = data['samples'][0]
            has_accel = 'accel' in sample
            has_jerk = 'jerk' in sample
            
            if has_accel and has_jerk:
                f.write("t_s,pos_rad,vel_rads,accel_rads2,jerk_rads3,current_A,pwm\n")
                for s in data['samples']:
                    f.write(f"{s['t']:.6f},{s['pos']:.8f},{s['vel']:.8f},"
                           f"{s.get('accel',0):.6f},{s.get('jerk',0):.6f},"
                           f"{s['current']:.6f},{s['pwm']}\n")
            else:
                f.write("t_s,pos_rad,vel_rads,current_A,pwm\n")
                for s in data['samples']:
                    f.write(f"{s['t']:.6f},{s['pos']:.8f},{s['vel']:.8f},"
                           f"{s['current']:.6f},{s['pwm']}\n")
        print(f"  → {path}")

    def _save_current_csv(self, motor, data, timestamp):
        """Guarda datos de perfil de corriente."""
        dir_path = os.path.join(self.output_dir, timestamp)
        os.makedirs(dir_path, exist_ok=True)
        
        path = os.path.join(dir_path, f"current_{motor}.csv")
        with open(path, 'w') as f:
            f.write("direccion,pwm,vel_rads,corriente_R_A,corriente_L_A\n")
            for d in data['fwd']:
                f.write(f"FWD,{d['pwm']:.4f},{d['vel']:.6f},{d['curr_R']:.6f},{d['curr_L']:.6f}\n")
            for d in data['rev']:
                f.write(f"REV,{d['pwm']:.4f},{d['vel']:.6f},{d['curr_R']:.6f},{d['curr_L']:.6f}\n")
        print(f"  → {path}")

    def _generate_plots(self, timestamp):
        """Genera gráficas de los datos recolectados."""
        dir_path = os.path.join(self.output_dir, timestamp)
        
        # --- Gráfica 1: Barrido PWM (velocidad vs PWM) ---
        for motor, data in self.sweep_data.items():
            if not data['raw']:
                continue
            
            fig, axes = plt.subplots(2, 2, figsize=(12, 8))
            fig.suptitle(f'Caracterización Motor {motor} - Barrido PWM', fontsize=14)
            
            times = [s['t'] for s in data['raw']]
            vels = [s['vel'] for s in data['raw']]
            pwms = [s['pwm'] for s in data['raw']]
            currents = [s['current'] for s in data['raw']]
            pos = [s['pos'] for s in data['raw']]
            
            # Velocidad en el tiempo
            axes[0,0].plot(times, vels)
            axes[0,0].set_xlabel('Tiempo [s]')
            axes[0,0].set_ylabel('Velocidad [rad/s]')
            axes[0,0].grid(True)
            axes[0,0].set_title('Velocidad vs Tiempo')
            
            # PWM en el tiempo
            axes[0,1].plot(times, pwms)
            axes[0,1].set_xlabel('Tiempo [s]')
            axes[0,1].set_ylabel('PWM [0-255]')
            axes[0,1].grid(True)
            axes[0,1].set_title('PWM vs Tiempo')
            
            # Velocidad vs PWM
            # Agrupar por PWM y promediar
            pwm_vals = sorted(set(pwms))
            vel_means = [np.mean([v for p, v in zip(pwms, vels) if p == pv]) for pv in pwm_vals]
            vel_stds = [np.std([v for p, v in zip(pwms, vels) if p == pv]) for pv in pwm_vals]
            
            axes[1,0].errorbar(pwm_vals, vel_means, yerr=vel_stds, fmt='o-', capsize=4)
            axes[1,0].set_xlabel('PWM [0-255]')
            axes[1,0].set_ylabel('Velocidad [rad/s]')
            axes[1,0].grid(True)
            axes[1,0].set_title('Velocidad Estacionaria vs PWM')
            
            # Corriente vs velocidad
            axes[1,1].plot(vels, currents, '.', alpha=0.5)
            axes[1,1].set_xlabel('Velocidad [rad/s]')
            axes[1,1].set_ylabel('Corriente [A]')
            axes[1,1].grid(True)
            axes[1,1].set_title('Corriente vs Velocidad')
            
            plt.tight_layout()
            path = os.path.join(dir_path, f"grafica_sweep_{motor}.png")
            fig.savefig(path, dpi=150)
            print(f"  → {path}")
            plt.close(fig)
        
        # --- Gráfica 2: Respuesta al escalón ---
        for motor, data in self.step_data.items():
            if not data['samples']:
                continue
            
            fig, axes = plt.subplots(3, 1, figsize=(10, 10))
            fig.suptitle(f'Respuesta al Escalón - Motor {motor} (PWM={data["pwm"]*100:.0f}%)', 
                        fontsize=14)
            
            times = [s['t'] for s in data['samples']]
            vels = [s['vel'] for s in data['samples']]
            pos = [s['pos'] for s in data['samples']]
            currents = [s['current'] for s in data['samples']]
            
            axes[0].plot(times, pos)
            axes[0].set_ylabel('Posición [rad]')
            axes[0].grid(True)
            axes[0].set_title('Posición')
            
            axes[1].plot(times, vels)
            axes[1].set_ylabel('Velocidad [rad/s]')
            axes[1].grid(True)
            axes[1].set_title('Velocidad')
            
            axes[2].plot(times, currents)
            axes[2].set_xlabel('Tiempo [s]')
            axes[2].set_ylabel('Corriente [A]')
            axes[2].grid(True)
            axes[2].set_title('Corriente')
            
            plt.tight_layout()
            path = os.path.join(dir_path, f"grafica_step_{motor}_pwm{data['pwm']:.0%}.png")
            fig.savefig(path, dpi=150)
            print(f"  → {path}")
            plt.close(fig)
        
        # --- Gráfica 3: Velocidad, Aceleración, Jerk (si hay datos) ---
        for motor, data in self.step_data.items():
            if not data['samples'] or 'accel' not in data['samples'][0]:
                continue
            
            fig, axes = plt.subplots(3, 1, figsize=(10, 10))
            fig.suptitle(f'Cinématica - Motor {motor} (PWM={data["pwm"]*100:.0f}%)', fontsize=14)
            
            times = [s['t'] for s in data['samples']]
            vels = [s['vel'] for s in data['samples']]
            accels = [s.get('accel', 0) for s in data['samples']]
            jerks = [s.get('jerk', 0) for s in data['samples']]
            
            axes[0].plot(times, vels)
            axes[0].set_ylabel('Velocidad [rad/s]')
            axes[0].grid(True)
            axes[0].set_title(f'Velocidad (máx: {max(vels):.3f} rad/s, '
                            f'estac: {np.mean(vels[-50:]):.3f} rad/s)')
            
            axes[1].plot(times, accels)
            axes[1].axhline(0, color='gray', linestyle='--')
            axes[1].set_ylabel('Aceleración [rad/s²]')
            axes[1].grid(True)
            axes[1].set_title(f'Aceleración (máx: {max(accels):.1f}, '
                            f'mín: {min(accels):.1f} rad/s²)')
            
            axes[2].plot(times, jerks)
            axes[2].axhline(0, color='gray', linestyle='--')
            axes[2].set_xlabel('Tiempo [s]')
            axes[2].set_ylabel('Jerk [rad/s³]')
            axes[2].grid(True)
            axes[2].set_title(f'Jerk (máx: {max(jerks):.1f}, '
                            f'mín: {min(jerks):.1f} rad/s³)')
            
            plt.tight_layout()
            path = os.path.join(dir_path, f"grafica_cinematica_{motor}_pwm{data['pwm']:.0%}.png")
            fig.savefig(path, dpi=150)
            print(f"  → {path}")
            plt.close(fig)
        
        # --- Gráfica 4: Perfil de corriente ---
        for motor, data in self.current_data.items():
            if not data['fwd'] and not data['rev']:
                continue
            
            fig, axes = plt.subplots(1, 2, figsize=(12, 5))
            fig.suptitle(f'Perfil de Corriente - Motor {motor}', fontsize=14)
            
            for dir_name, dir_data, marker, color in [
                ('FWD', data['fwd'], 'o', 'blue'),
                ('REV', data['rev'], 's', 'red')
            ]:
                if not dir_data:
                    continue
                pwms = [d['pwm'] for d in dir_data]
                vels = [d['vel'] for d in dir_data]
                currs = [(d['curr_R'] + d['curr_L']) / 2 for d in dir_data]
                
                axes[0].plot(pwms, vels, f'{marker}-', color=color, label=f'{dir_name}')
                axes[1].plot(vels, currs, f'{marker}-', color=color, label=f'{dir_name}')
            
            axes[0].set_xlabel('PWM [0-1]')
            axes[0].set_ylabel('Velocidad [rad/s]')
            axes[0].grid(True)
            axes[0].legend()
            axes[0].set_title('Velocidad vs PWM')
            
            axes[1].set_xlabel('Velocidad [rad/s]')
            axes[1].set_ylabel('Corriente promedio [A]')
            axes[1].grid(True)
            axes[1].legend()
            axes[1].set_title('Corriente vs Velocidad')
            
            plt.tight_layout()
            path = os.path.join(dir_path, f"grafica_current_{motor}.png")
            fig.savefig(path, dpi=150)
            print(f"  → {path}")
            plt.close(fig)


# ============================================================================
# INTERFAZ DE USUARIO
# ============================================================================

def print_menu():
    print("\n" + "="*60)
    print("COMANDOS DISPONIBLES")
    print("="*60)
    print("  sweep base      - Barrido PWM motor BASE")
    print("  sweep codo      - Barrido PWM motor CODO")
    print("  step base [pwm] - Escalón BASE (pwm: 0.1-1.0, def: 0.7)")
    print("  step codo [pwm] - Escalón CODO")
    print("  current base    - Perfil corriente BASE")
    print("  current codo    - Perfil corriente CODO")
    print("  full            - Calibración COMPLETA (todo)")
    print("  stop            - Parada de emergencia")
    print("  status          - Estado actual")
    print("  plot            - Generar gráficas manualmente")
    print("  save            - Guardar datos manualmente")
    print("  quit            - Salir")
    print("="*60)

def main():
    parser = argparse.ArgumentParser(description='Capturador de calibración Robot 2R')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Puerto serial (def: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=921600,
                       help='Baud rate (def: 921600)')
    args = parser.parse_args()

    print(f"CAPTURADOR DE CALIBRACIÓN - ROBOT 2R")
    print(f"Puerto: {args.port} @ {args.baud} baud")
    print()

    capture = CalibrationCapture(args.port, args.baud)
    if not capture.connect():
        print("¿Desea intentar con otro puerto?")
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyS*')
        print(f"Puertos encontrados: {ports}")
        return

    capture.start_capture()
    
    print_menu()
    
    try:
        while True:
            cmd = input("\n> ").strip().lower()
            
            if cmd == 'quit' or cmd == 'exit':
                break
            elif cmd == 'help' or cmd == '?':
                print_menu()
            elif cmd == 'sweep base':
                capture.send_command('SWEEP_BASE')
            elif cmd == 'sweep codo':
                capture.send_command('SWEEP_CODO')
            elif cmd.startswith('step base'):
                parts = cmd.split()
                pwm = parts[2] if len(parts) > 2 else None
                capture.send_command(f'STEP_BASE {pwm}' if pwm else 'STEP_BASE')
            elif cmd.startswith('step codo'):
                parts = cmd.split()
                pwm = parts[2] if len(parts) > 2 else None
                capture.send_command(f'STEP_CODO {pwm}' if pwm else 'STEP_CODO')
            elif cmd == 'current base':
                capture.send_command('CURRENT_BASE')
            elif cmd == 'current codo':
                capture.send_command('CURRENT_CODO')
            elif cmd == 'full':
                capture.send_command('FULL_CALIB')
            elif cmd == 'stop':
                capture.send_command('STOP')
            elif cmd == 'status':
                capture.send_command('STATUS')
            elif cmd == 'save':
                capture._save_all_data()
            elif cmd == 'plot' and plt:
                capture._generate_plots(datetime.now().strftime("%Y%m%d_%H%M%S"))
            elif cmd:
                print(f"Comando no reconocido: {cmd}")
                
    except KeyboardInterrupt:
        print("\n\nInterrupción detectada.")
    finally:
        capture.send_command('STOP')
        capture.disconnect()
        print("¡Hasta luego!")


if __name__ == '__main__':
    main()
