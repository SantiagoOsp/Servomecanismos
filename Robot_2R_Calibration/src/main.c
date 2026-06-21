// ============================================================================
// PROGRAMA DE CARACTERIZACIÓN FINA DE MOTORES - ROBOT 2R
// Firmware para ESP32 (fm-devkit)
// 
// Propósito: Realizar barridos de PWM, respuestas al escalón, perfiles de
// velocidad/aceleración/jerk y sensado de corriente para caracterizar
// finamente los motores DC del robot 2R.
//
// =====================================================================
// MCPWM (Motor Control PWM) de la ESP32
// =====================================================================
// Se usa el periférico MCPWM en vez de LEDC genérico porque ofrece:
//   ✔ Modulación independiente RPWM/LPWM para puente H
//   ✔ Pines de FAULT (opcionales) para parada de emergencia
//   ✔ Control de duty cycle en % (0.0 - 100.0)
//   ✔ Resolución completa a 20 kHz
//
// PROTECCIÓN DE SOBRECORRIENTE:
//   Los pines IS de los BTS7960 se conectan DIRECTAMENTE a los ADC
//   de la ESP32 (sin amplificadores ni comparadores externos).
//   El monitoreo se hace por SOFTWARE a través de una tarea dedicada
//   que lee la corriente cada 50ms y frena el motor si excede el umbral.
//
//   Tensión en pin IS = (I_motor / 8500) × 1000Ω (resistencia interna del módulo)
//   Motor BASE  (12V): 10A pico → 1.18V  → ADC mide bien
//   Motor CODO  (24V): 3.1A pico → 0.365V → ADC mide con resolución reducida
//
// =====================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm.h"     // MCPWM para control de motores
#include "driver/pcnt.h"      // Encoder counter
#include "driver/adc.h"       // ADC para corriente
#include "driver/gpio.h"      // GPIO pull-down para FAULT
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"        // NVS - almacenamiento persistente
#include "nvs.h"              // API de NVS
#include "sdkconfig.h"

// ============================================================================
// CONSTANTES DE CONFIGURACIÓN
// ============================================================================

#define TAG "CALIBRACION"

// ---- PINES PWM (MCPWM) ----
// MCPWM_UNIT_0 → Motor Base
//   MCPWM0A = RPWM base, MCPWM0B = LPWM base
#define BASE_RPWM_GPIO   32  // MCPWM0A
#define BASE_LPWM_GPIO   33  // MCPWM0B
#define BASE_REN_GPIO    25
#define BASE_LEN_GPIO    26

// MCPWM_UNIT_1 → Motor Codo
//   MCPWM1A = RPWM codo, MCPWM1B = LPWM codo
#define CODO_RPWM_GPIO   27  // MCPWM1A
#define CODO_LPWM_GPIO   14  // MCPWM1B
#define CODO_REN_GPIO    12
#define CODO_LEN_GPIO    13

// ---- PINES FAULT (OPCIONAL - Parada de emergencia por hardware) ----
// Si NO se usan, dejar sin conectar (pull-down interno los mantiene en LOW).
// Para activar: conectar salida de comparador externo (LM393) cuandose
// quiera protección por hardware (~1μs de reacción).
// PUESTOS POR DEFECTO EN LOW (desactivados) - la protección es por SOFTWARE.
#define BASE_FAULT_GPIO  23  // MCPWM_FAULT_0 (dejar NC o GND si no se usa)
#define CODO_FAULT_GPIO  22  // MCPWM_FAULT_1 (dejar NC o GND si no se usa)

// ---- PINES ENCODER ----
#define ENC_BASE_CHA     16
#define ENC_BASE_CHB     17
#define ENC_CODO_CHA     18
#define ENC_CODO_CHB     19

// ---- PINES ADC (Corriente) ----
#define BASE_R_IS        ADC1_CHANNEL_6  // GPIO 34
#define BASE_L_IS        ADC1_CHANNEL_7  // GPIO 35
#define CODO_R_IS        ADC1_CHANNEL_0  // GPIO 36 (ADC1_CH0 = GPIO36)
#define CODO_L_IS        ADC1_CHANNEL_3  // GPIO 39 (ADC1_CH3 = GPIO39)

// ---- Parámetros de PWM (MCPWM) ----
#define PWM_FREQ         20000           // 20 kHz (ultrasónico)
#define MCPWM_DUTY_MAX   1000            // Periodo en ticks del MCPWM (resolución efectiva)
// El duty cycle se maneja en % (0.0 a 100.0) con resolución de 1/MCPWM_DUTY_MAX

// ---- PCNT ----
#define PCNT_H_LIM       32767
#define PCNT_LO_LIM     -32767

// ---- Parámetros físicos de motores ----
#define BASE_GEAR_RATIO  218.0f
#define CODO_GEAR_RATIO  270.0f
#define ENCODER_PPR      11              // Pulsos por revolución del motor (encoder hall típico)
#define CPR              (4 * ENCODER_PPR)  // Cuadratura: 4x pulsos

// Resolución angular [rad/pulso] en la salida del gearbox
#define BASE_RAD_PP      (2.0f * M_PI / (CPR * BASE_GEAR_RATIO))
#define CODO_RAD_PP      (2.0f * M_PI / (CPR * CODO_GEAR_RATIO))

// ---- Sensado de corriente ----
#define ISENSE_RSENSE    1000.0f         // Resistencia de sense [Ω] en el módulo (típico 1kΩ)
#define ISENSE_KILIS     8500.0f         // Factor de escala BTS7960 (I_load / I_IS)
#define ADC_REF_VOLTAGE  3.3f            // Vref ESP32
#define ADC_12BIT_MAX    4095.0f

// ---- Configuración de calibración ----
#define SAMPLE_RATE      1000            // Hz (tasa de muestreo del lazo de control)
#define DT_S             (1.0f / SAMPLE_RATE)
#define DT_MS            (1000 / SAMPLE_RATE)
#define DT_US            (1000000 / SAMPLE_RATE)
#define NUM_PWM_STEPS    10              // Número de escalones PWM a probar
#define STEP_SETTLE_S    3.0f            // Segundos para estabilizar cada escalón
#define STEP_RECORD_S    2.0f            // Segundos de grabación por escalón

// ---- NVS (Persistencia) ----
#define NVS_NAMESPACE    "robot2r"
#define NVS_KEY_HOME_TH1 "home_th1"
#define NVS_KEY_HOME_TH2 "home_th2"
#define NVS_KEY_MIN_TH1  "min_th1"
#define NVS_KEY_MAX_TH1  "max_th1"
#define NVS_KEY_MIN_TH2  "min_th2"
#define NVS_KEY_MAX_TH2  "max_th2"
#define NVS_KEY_CALIB_OK "calib_done"



// ============================================================================
// ESTRUCTURAS DE DATOS
// ============================================================================

typedef struct {
    int64_t timestamp_us;
    int32_t encoder_raw;
    float   pos_rad;
    float   vel_rads;
    float   current_A;
    uint8_t pwm_duty;       // 0-255 (escala 8 bits para legibilidad)
} motor_sample_t;

typedef struct {
    float pwm_pct;          // 0.0 - 1.0
    float vel_steady_rads;
    float vel_std_rads;
    float current_steady_A;
    float accel_max_rads2;
    float jerk_max_rads3;
} calibration_point_t;

typedef struct {
    char nombre[16];
    // MCPWM
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t mcpwm_timer;
    mcpwm_io_signals_t mcpwm_a;   // RPWM
    mcpwm_io_signals_t mcpwm_b;   // LPWM
    mcpwm_fault_signal_t fault_sig;
    // GPIOs
    int rpwm_gpio;
    int lpwm_gpio;
    int fault_gpio;
    int ren_gpio;
    int len_gpio;
    // Encoder
    int enc_cha, enc_chb;
    pcnt_unit_t pcnt_unit;
    // ADC
    adc1_channel_t adc_r_is;
    adc1_channel_t adc_l_is;
    esp_adc_cal_characteristics_t *adc_chars;
    // Estado
    float gear_ratio;
    float rad_per_pulse;
    float pos_offset_rad;
    int32_t last_enc_raw;
    float vel_filtered;
    float i_load_max;             // Umbral de corriente para auto-brake
    volatile int fault_triggered; // Flag de fault
} motor_t;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

static motor_t motor_base = {
    .nombre = "BASE",
    .mcpwm_unit = MCPWM_UNIT_0,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_a = MCPWM0A,
    .mcpwm_b = MCPWM0B,
    .fault_sig = MCPWM_SELECT_F0,
    .rpwm_gpio = BASE_RPWM_GPIO,
    .lpwm_gpio = BASE_LPWM_GPIO,
    .fault_gpio = BASE_FAULT_GPIO,
    .ren_gpio = BASE_REN_GPIO,
    .len_gpio = BASE_LEN_GPIO,
    .enc_cha = ENC_BASE_CHA,
    .enc_chb = ENC_BASE_CHB,
    .pcnt_unit = PCNT_UNIT_0,
    .adc_r_is = BASE_R_IS,
    .adc_l_is = BASE_L_IS,
    .gear_ratio = BASE_GEAR_RATIO,
    .rad_per_pulse = BASE_RAD_PP,
    .pos_offset_rad = 0.0f,
    .last_enc_raw = 0,
    .vel_filtered = 0.0f,
    .i_load_max = 8.0f,       // 8A → salta antes del pico de 10A
    .fault_triggered = 0,
    .adc_chars = NULL
};

static motor_t motor_codo = {
    .nombre = "CODO",
    .mcpwm_unit = MCPWM_UNIT_1,
    .mcpwm_timer = MCPWM_TIMER_0,
    .mcpwm_a = MCPWM1A,
    .mcpwm_b = MCPWM1B,
    .fault_sig = MCPWM_SELECT_F0,
    .rpwm_gpio = CODO_RPWM_GPIO,
    .lpwm_gpio = CODO_LPWM_GPIO,
    .fault_gpio = CODO_FAULT_GPIO,
    .ren_gpio = CODO_REN_GPIO,
    .len_gpio = CODO_LEN_GPIO,
    .enc_cha = ENC_CODO_CHA,
    .enc_chb = ENC_CODO_CHB,
    .pcnt_unit = PCNT_UNIT_1,
    .adc_r_is = CODO_R_IS,
    .adc_l_is = CODO_L_IS,
    .gear_ratio = CODO_GEAR_RATIO,
    .rad_per_pulse = CODO_RAD_PP,
    .pos_offset_rad = 0.0f,
    .last_enc_raw = 0,
    .vel_filtered = 0.0f,
    .i_load_max = 2.5f,       // 2.5A → salta antes del pico de 3.1A
    .fault_triggered = 0,
    .adc_chars = NULL
};

static volatile int64_t g_t0_us = 0;
static volatile int g_running = 0;

// Cola para enviar datos al PC
static QueueHandle_t data_queue = NULL;
#define QUEUE_SIZE 512

// ============================================================================
// PROTOTIPOS
// ============================================================================
void motor_init(motor_t *m);
void motor_set_pwm(motor_t *m, float duty_cycle);
void motor_brake(motor_t *m);
void motor_coast(motor_t *m);
void motor_clear_fault(motor_t *m);
float motor_read_current(motor_t *m, adc1_channel_t ch);
int32_t motor_read_encoder(motor_t *m);
void motor_reset_encoder(motor_t *m);
void check_current_and_brake(motor_t *m);
void calibrate_pwm_sweep(motor_t *m, calibration_point_t *results, int *n_points);
void calibrate_step_response(motor_t *m, float pwm_pct, float duration_s);
void calibrate_current_profile(motor_t *m);
void data_log_task(void *pv);
void serial_cmd_task(void *pv);
void current_monitor_task(void *pv);
void nvs_save_home(float th1, float th2);
int  nvs_load_home(float *th1, float *th2);
void nvs_save_calib_done(void);
int  nvs_is_calib_done(void);
void parse_and_execute(char *line);

// ============================================================================
// INICIALIZACIÓN DE MOTOR (MCPWM + ENCODER + ADC + FAULT)
// ============================================================================

void motor_init(motor_t *m) {
    ESP_LOGI(TAG, "Inicializando motor %s con MCPWM...", m->nombre);

    // ---- 1. Configurar pines de enable como OUTPUT ----
    gpio_set_direction(m->ren_gpio, GPIO_MODE_OUTPUT);
    gpio_set_direction(m->len_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(m->ren_gpio, 1);
    gpio_set_level(m->len_gpio, 1);

    // ---- 2. Inicializar MCPWM ----
    // Configuración: 20 kHz, up-counter, duty en % (0-100)
    mcpwm_config_t pwm_config = {
        .frequency    = PWM_FREQ,
        .cmpr_a       = 0,         // RPWM duty inicial = 0%
        .cmpr_b       = 0,         // LPWM duty inicial = 0%
        .duty_mode    = MCPWM_DUTY_MODE_0,  // duty activo HIGH
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Inicializar MCPWM unit con timer 0
    // Esto configura los GPIOs, timers y operadores
    mcpwm_gpio_init(m->mcpwm_unit, m->mcpwm_a, m->rpwm_gpio);
    mcpwm_gpio_init(m->mcpwm_unit, m->mcpwm_b, m->lpwm_gpio);
    mcpwm_init(m->mcpwm_unit, m->mcpwm_timer, &pwm_config);

ESP_LOGI(TAG, "  MCPWM %s: %d Hz, canales GPIO %d, %d",
             m->nombre, PWM_FREQ, m->rpwm_gpio, m->lpwm_gpio);

    // ---- 3. FAULT (OPCIONAL - pull-down = inactivo) ----
    // En ESP-IDF 5.5+ la API legacy de fault está obsoleta.
    // La protección es por SOFTWARE vía ADC (sin hardware fault).
    gpio_set_pull_mode(m->fault_gpio, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(m->fault_gpio, GPIO_MODE_INPUT);
    ESP_LOGI(TAG, "  FAULT GPIO %d configurado (pull-down, inactivo)", m->fault_gpio);

    // ---- 4. Configurar PCNT (encoder) ----
    pcnt_config_t pcnt_cfg = {
        .pulse_gpio_num = m->enc_cha,
        .ctrl_gpio_num  = m->enc_chb,
        .channel        = PCNT_CHANNEL_0,
        .unit           = m->pcnt_unit,
        .pos_mode       = PCNT_COUNT_INC,    // A↑: incrementa
        .neg_mode       = PCNT_COUNT_DEC,    // A↓: decrementa
        .lctrl_mode     = PCNT_MODE_REVERSE, // B=1: invierte dirección
        .hctrl_mode     = PCNT_MODE_KEEP,    // B=0: mantiene dirección
        .counter_h_lim  = PCNT_H_LIM,
        .counter_l_lim  = PCNT_LO_LIM,
    };
    pcnt_unit_config(&pcnt_cfg);
    pcnt_counter_pause(m->pcnt_unit);
    pcnt_counter_clear(m->pcnt_unit);
    pcnt_counter_resume(m->pcnt_unit);

    // ---- 5. Caracterizar ADC para corriente ----
    m->adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 0, m->adc_chars);

    ESP_LOGI(TAG, "Motor %s inicializado (MCPWM + FAULT + PCNT + ADC).", m->nombre);
}

// ============================================================================
// CONTROL DE MOTOR VÍA MCPWM
// ============================================================================

void motor_set_pwm(motor_t *m, float duty_cycle) {
    // duty_cycle: -1.0 (máx reversa) → +1.0 (máx forward)
    // 0.0 → brake activo (ambos PWMs HIGH)
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    if (duty_cycle < -1.0f) duty_cycle = -1.0f;

    float duty_pct = fabsf(duty_cycle) * 100.0f;  // 0-100%

    if (duty_cycle > 0) {
        // Forward: RPWM = PWM%, LPWM = 0%
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, duty_pct);
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, 0.0f);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    } else if (duty_cycle < 0) {
        // Reverse: RPWM = 0%, LPWM = PWM%
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, duty_pct);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    } else {
        // Brake: ambos al 100% (cortocircuito por low-sides del BTS7960)
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, 100.0f);
        mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, 100.0f);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

void motor_brake(motor_t *m) {
    // Freno activo: ambos PWMs al 100%
    gpio_set_level(m->ren_gpio, 1);
    gpio_set_level(m->len_gpio, 1);
    mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, 100.0f);
    mcpwm_set_duty(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, 100.0f);
    mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(m->mcpwm_unit, m->mcpwm_timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    m->fault_triggered = 0;  // limpia flag de fault
}

void motor_coast(motor_t *m) {
    // Rueda libre: deshabilitar enables
    gpio_set_level(m->ren_gpio, 0);
    gpio_set_level(m->len_gpio, 0);
}

void motor_clear_fault(motor_t *m) {
    // En IDF 5.5+ la API legacy mcpwm_brake_clear no existe.
    // Simplemente reseteamos el flag y reaplicamos PWM=0.
    m->fault_triggered = 0;
    motor_set_pwm(m, 0.0f);
    ESP_LOGI(TAG, "Fault limpiado en motor %s", m->nombre);
}

// ============================================================================
// MONITOREO DE CORRIENTE POR SOFTWARE (protección principal)
// ============================================================================
// Los pines IS de los BTS7960 se conectan DIRECTAMENTE a los ADC de la
// ESP32. Una tarea periódica (20 Hz) lee la corriente y frena el motor
// si excede el umbral configurado (i_load_max).
//
// Esta es la ÚNICA protección activa (no se usan comparadores externos).
// La reacción es ~50ms (suficiente para calibración controlada).
// ===========================================================================

void check_current_and_brake(motor_t *m) {
    float i_r = motor_read_current(m, m->adc_r_is);
    float i_l = motor_read_current(m, m->adc_l_is);
    float i_max = (i_r > i_l) ? i_r : i_l;
    
    if (i_max > m->i_load_max) {
        ESP_LOGW(TAG, "⚠ SOBRECORRIENTE %s: %.2f A (límite %.2f A)!",
                 m->nombre, i_max, m->i_load_max);
        motor_brake(m);
        m->fault_triggered = 1;
    }
}

// ============================================================================
// LECTURA DE SENSORES
// ============================================================================

int32_t motor_read_encoder(motor_t *m) {
    int16_t count;
    pcnt_get_counter_value(m->pcnt_unit, &count);
    return (int32_t)count;
}

void motor_reset_encoder(motor_t *m) {
    pcnt_counter_pause(m->pcnt_unit);
    pcnt_counter_clear(m->pcnt_unit);
    pcnt_counter_resume(m->pcnt_unit);
    m->last_enc_raw = 0;
    m->pos_offset_rad = 0.0f;
}

float motor_read_current(motor_t *m, adc1_channel_t ch) {
    // Lee voltaje del pin IS y lo convierte a corriente de motor
    uint32_t adc_raw = adc1_get_raw(ch);
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_raw, m->adc_chars);
    float v_is = voltage_mv / 1000.0f;  // V
    
    // I_IS = V_IS / R_SENSE
    float i_is = v_is / ISENSE_RSENSE;
    
    // I_load = I_IS * K_ILIS
    float i_load = i_is * ISENSE_KILIS;
    
    return i_load;
}

// ============================================================================
// RUTINAS DE CALIBRACIÓN
// ============================================================================

// --- BARRIDO DE PWM (velocidad en estado estacionario) ---
void calibrate_pwm_sweep(motor_t *m, calibration_point_t *results, int *n_points) {
    ESP_LOGI(TAG, "=== INICIANDO BARRIDO PWM - %s ===", m->nombre);
    printf("CMD:SWEEP_START:%s\n", m->nombre);
    
    float pwm_values[NUM_PWM_STEPS];
    for (int i = 0; i < NUM_PWM_STEPS; i++) {
        pwm_values[i] = 0.1f + 0.9f * (float)i / (NUM_PWM_STEPS - 1);
    }
    
    int idx = 0;
    for (int i = 0; i < NUM_PWM_STEPS; i++) {
        float pwm_f = pwm_values[i];
        
        ESP_LOGI(TAG, "  PWM = %.1f%% (%.0f/255)", pwm_f * 100, pwm_f * 255);
        printf("CMD:SWEEP_POINT:%s:%.4f\n", m->nombre, pwm_f);
        
        // Aplicar PWM en sentido positivo
        motor_set_pwm(m, pwm_f);
        vTaskDelay(pdMS_TO_TICKS(500));  // pequeño retardo para arranque
        
        // --- Fase de estabilización ---
        float vel_sum = 0, vel_sum2 = 0;
        float curr_sum = 0;
        int n_samples = 0;
        int settle_samples = (int)(STEP_SETTLE_S * SAMPLE_RATE);
        
        for (int s = 0; s < settle_samples; s++) {
            int32_t enc = motor_read_encoder(m);
            float vel = (enc - m->last_enc_raw) * m->rad_per_pulse / DT_S;
            m->vel_filtered = 0.8f * m->vel_filtered + 0.2f * vel;
            m->last_enc_raw = enc;
            
            // Solo tomar muestras después de estabilizar un poco
            if (s > settle_samples / 3) {
                vel_sum += m->vel_filtered;
                vel_sum2 += m->vel_filtered * m->vel_filtered;
                curr_sum += motor_read_current(m, m->adc_r_is);
                n_samples++;
            }
            
            // Enviar datos en tiempo real para monitoreo
            int64_t t = esp_timer_get_time();
            printf("DATA:%s:%lld:%.6f:%.6f:%.6f:%u\n",
                   m->nombre, t, 
                   enc * m->rad_per_pulse,  // posición
                   m->vel_filtered,          // velocidad filtrada
                   motor_read_current(m, m->adc_r_is),
                   (unsigned)(pwm_f * 255));
            
            vTaskDelay(pdMS_TO_TICKS(DT_MS));
        }
        
        if (n_samples > 0) {
            float vel_mean = vel_sum / n_samples;
            float vel_var = (vel_sum2 / n_samples) - (vel_mean * vel_mean);
            if (vel_var < 0) vel_var = 0;
            
            results[idx].pwm_pct = pwm_f;
            results[idx].vel_steady_rads = vel_mean;
            results[idx].vel_std_rads = sqrtf(vel_var);
            results[idx].current_steady_A = curr_sum / n_samples;
            results[idx].accel_max_rads2 = 0;
            results[idx].jerk_max_rads3 = 0;
            idx++;
        }
        
        // Freno y pausa entre puntos
        motor_brake(m);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    *n_points = idx;
    
    // Enviar resultados al PC
    printf("CMD:SWEEP_RESULTS:%s\n", m->nombre);
    for (int i = 0; i < idx; i++) {
        printf("RES:%.4f:%.6f:%.6f:%.6f\n",
               results[i].pwm_pct,
               results[i].vel_steady_rads,
               results[i].vel_std_rads,
               results[i].current_steady_A);
    }
    printf("CMD:SWEEP_END:%s\n", m->nombre);
    ESP_LOGI(TAG, "=== BARRIDO PWM %s COMPLETADO (%d puntos) ===", m->nombre, idx);
    
    motor_brake(m);
}

// --- RESPUESTA AL ESCALÓN (dinámica: aceleración y jerk) ---
void calibrate_step_response(motor_t *m, float pwm_pct, float duration_s) {
    ESP_LOGI(TAG, "=== RESPUESTA AL ESCALÓN - %s (PWM=%.1f%%) ===", m->nombre, pwm_pct);
    printf("CMD:STEP_START:%s:%.4f:%.2f\n", m->nombre, pwm_pct, duration_s);
    
    motor_reset_encoder(m);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    int n_samples = (int)(duration_s * SAMPLE_RATE);
    
    // Variables para velocidad y aceleración con filtrado
    float vel_filt = 0, accel_filt = 0, jerk_filt = 0;
    float vel_prev = 0;
    int32_t enc_prev = 0;
    
    // Almacenar para enviar después (evitar saturar UART)
    motor_sample_t *buffer = malloc(n_samples * sizeof(motor_sample_t));
    if (!buffer) {
        ESP_LOGE(TAG, "Error de memoria");
        return;
    }
    
    // Aplicar escalón
    motor_set_pwm(m, pwm_pct);
    int64_t t_start = esp_timer_get_time();
    
    for (int i = 0; i < n_samples; i++) {
        int64_t t_now = esp_timer_get_time();
        int32_t enc = motor_read_encoder(m);
        float pos_rad = enc * m->rad_per_pulse;
        float vel = (enc - enc_prev) * m->rad_per_pulse / DT_S;
        enc_prev = enc;
        
        // Filtro pasa-bajos para velocidad
        vel_filt = 0.85f * vel_filt + 0.15f * vel;
        
        // Aceleración (derivada de velocidad filtrada)
        float accel = (vel_filt - vel_prev) / DT_S;
        vel_prev = vel_filt;
        accel_filt = 0.8f * accel_filt + 0.2f * accel;
        
        // Jerk (derivada de aceleración)
        jerk_filt = 0.7f * jerk_filt + 0.3f * ((accel_filt - (i > 0 ? 
            (buffer[i-1].vel_rads - (i>1 ? buffer[i-2].vel_rads : 0))/DT_S : 0)) / DT_S);
        
        float current = motor_read_current(m, m->adc_r_is);
        
        buffer[i].timestamp_us = t_now - t_start;
        buffer[i].encoder_raw = enc;
        buffer[i].pos_rad = pos_rad;
        buffer[i].vel_rads = vel_filt;
        buffer[i].current_A = current;
        buffer[i].pwm_duty = (uint8_t)(pwm_pct * 255);
        
        // Enviar en tiempo real (cada 5 muestras para no saturar)
        if (i % 5 == 0) {
            printf("DATA:STEP:%s:%lld:%.6f:%.6f:%.6f:%.2f:%.2f:%u\n",
                   m->nombre, t_now - t_start,
                   pos_rad, vel_filt, accel_filt, jerk_filt, current,
                   (unsigned)(pwm_pct * 255));
        }
        
        vTaskDelay(pdMS_TO_TICKS(DT_MS));
    }
    
    // Freno
    motor_brake(m);
    
    // Enviar resultados completos
    printf("CMD:STEP_DATA:%s:%d\n", m->nombre, n_samples);
    for (int i = 0; i < n_samples; i += 2) {  // diezmado 2:1 para no saturar
        printf("DATA:STEP_FULL:%s:%lld:%.6f:%.6f:%.6f:%u\n",
               m->nombre, buffer[i].timestamp_us,
               buffer[i].pos_rad, buffer[i].vel_rads, buffer[i].current_A,
               buffer[i].pwm_duty);
    }
    printf("CMD:STEP_END:%s\n", m->nombre);
    
    free(buffer);
    ESP_LOGI(TAG, "=== RESPUESTA AL ESCALÓN %s COMPLETADA ===", m->nombre);
}

// --- CARACTERIZACIÓN DE CORRIENTE vs VELOCIDAD ---
void calibrate_current_profile(motor_t *m) {
    ESP_LOGI(TAG, "=== PERFIL DE CORRIENTE - %s ===", m->nombre);
    printf("CMD:CURRENT_START:%s\n", m->nombre);
    
    // Barrer varios PWM y medir corriente vs velocidad
    float pwm_vals[] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    int n_pts = sizeof(pwm_vals) / sizeof(pwm_vals[0]);
    
    for (int i = 0; i < n_pts; i++) {
        float pwm = pwm_vals[i];
        motor_set_pwm(m, pwm);
        vTaskDelay(pdMS_TO_TICKS(2000));  // esperar estado estacionario
        
        float curr_r = 0, curr_l = 0;
        float vel_sum = 0;
        int n = 0;
        
        for (int s = 0; s < 200; s++) {
            int32_t enc = motor_read_encoder(m);
            float vel = (enc - m->last_enc_raw) * m->rad_per_pulse / DT_S;
            m->vel_filtered = 0.9f * m->vel_filtered + 0.1f * vel;
            m->last_enc_raw = enc;
            
            vel_sum += m->vel_filtered;
            curr_r += motor_read_current(m, m->adc_r_is);
            curr_l += motor_read_current(m, m->adc_l_is);
            n++;
            vTaskDelay(pdMS_TO_TICKS(DT_MS));
        }
        
        float vel_avg = vel_sum / n;
        float curr_r_avg = curr_r / n;
        float curr_l_avg = curr_l / n;
        
        printf("DATA:CURRENT:%s:%.4f:%.6f:%.6f:%.6f\n",
               m->nombre, pwm, vel_avg, curr_r_avg, curr_l_avg);
        
        motor_set_pwm(m, -pwm);  // reversa
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        curr_r = 0; curr_l = 0; vel_sum = 0; n = 0;
        for (int s = 0; s < 200; s++) {
            int32_t enc = motor_read_encoder(m);
            float vel = (enc - m->last_enc_raw) * m->rad_per_pulse / DT_S;
            m->vel_filtered = 0.9f * m->vel_filtered + 0.1f * vel;
            m->last_enc_raw = enc;
            
            vel_sum += m->vel_filtered;
            curr_r += motor_read_current(m, m->adc_r_is);
            curr_l += motor_read_current(m, m->adc_l_is);
            n++;
            vTaskDelay(pdMS_TO_TICKS(DT_MS));
        }
        
        vel_avg = vel_sum / n;
        curr_r_avg = curr_r / n;
        curr_l_avg = curr_l / n;
        
        printf("DATA:CURRENT:REV_%s:%.4f:%.6f:%.6f:%.6f\n",
               m->nombre, pwm, vel_avg, curr_r_avg, curr_l_avg);
        
        motor_brake(m);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    printf("CMD:CURRENT_END:%s\n", m->nombre);
    ESP_LOGI(TAG, "=== PERFIL CORRIENTE %s COMPLETADO ===", m->nombre);
}

// ============================================================================
// TAREA DE PROCESAMIENTO DE COMANDOS SERIALES (PC -> ESP32)
// ============================================================================

void serial_cmd_task(void *pv) {
    char line[256];
    int pos = 0;
    char c;
    
    ESP_LOGI(TAG, "Tarea de comandos seriales iniciada. Envíe comandos desde el PC.");
    printf("\n=== CALIBRADOR DE MOTORES ROBOT 2R ===\n");
    printf("Comandos de calibración:\n");
    printf("  SWEEP_BASE/CODO  - Barrido PWM\n");
    printf("  STEP_BASE/CODO p - Respuesta al escalón\n");
    printf("  CURRENT_BASE/CODO- Perfil corriente\n");
    printf("  FULL_CALIB       - Calibración completa\n");
    printf("Comandos de persistencia (NVS):\n");
    printf("  SAVE_HOME        - Guarda posición Home actual\n");
    printf("  LOAD_HOME        - Carga Home guardado\n");
    printf("Control:\n");
    printf("  STOP / STATUS\n");
    printf("=======================================\n");
    
    while (1) {
        c = getchar();
        if (c == '\r' || c == '\n') {
            if (pos > 0) {
                line[pos] = '\0';
                parse_and_execute(line);
                pos = 0;
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
}

void parse_and_execute(char *line) {
    // Eliminar espacios al inicio
    while (*line == ' ') line++;
    
    if (strcmp(line, "SWEEP_BASE") == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        g_running = 1;
        calibration_point_t results[20];
        int n = 0;
        calibrate_pwm_sweep(&motor_base, results, &n);
        g_running = 0;
    }
    else if (strcmp(line, "SWEEP_CODO") == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        g_running = 1;
        calibration_point_t results[20];
        int n = 0;
        calibrate_pwm_sweep(&motor_codo, results, &n);
        g_running = 0;
    }
    else if (strncmp(line, "STEP_BASE", 9) == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        float pwm = 0.7f;
        if (strlen(line) > 10) pwm = atof(line + 10);
        if (pwm < 0.1f) pwm = 0.1f;
        if (pwm > 1.0f) pwm = 1.0f;
        g_running = 1;
        calibrate_step_response(&motor_base, pwm, 3.0f);
        g_running = 0;
    }
    else if (strncmp(line, "STEP_CODO", 9) == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        float pwm = 0.7f;
        if (strlen(line) > 10) pwm = atof(line + 10);
        if (pwm < 0.1f) pwm = 0.1f;
        if (pwm > 1.0f) pwm = 1.0f;
        g_running = 1;
        calibrate_step_response(&motor_codo, pwm, 3.0f);
        g_running = 0;
    }
    else if (strcmp(line, "CURRENT_BASE") == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        g_running = 1;
        calibrate_current_profile(&motor_base);
        g_running = 0;
    }
    else if (strcmp(line, "CURRENT_CODO") == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        g_running = 1;
        calibrate_current_profile(&motor_codo);
        g_running = 0;
    }
    else if (strcmp(line, "FULL_CALIB") == 0) {
        if (g_running) { printf("CMD:BUSY\n"); return; }
        g_running = 1;
        printf("CMD:FULL_CALIB_START\n");
        
        calibration_point_t res_base[20], res_codo[20];
        int nb = 0, nc = 0;
        
        calibrate_pwm_sweep(&motor_base, res_base, &nb);
        calibrate_pwm_sweep(&motor_codo, res_codo, &nc);
        
        calibrate_step_response(&motor_base, 0.7f, 3.0f);
        calibrate_step_response(&motor_codo, 0.7f, 3.0f);
        
        calibrate_step_response(&motor_base, 0.4f, 3.0f);
        calibrate_step_response(&motor_codo, 0.4f, 3.0f);
        
        calibrate_current_profile(&motor_base);
        calibrate_current_profile(&motor_codo);
        
        printf("CMD:FULL_CALIB_END\n");
        g_running = 0;
    }
    else if (strcmp(line, "STOP") == 0) {
        ESP_LOGW(TAG, "*** PARADA DE EMERGENCIA ***");
        motor_brake(&motor_base);
        motor_brake(&motor_codo);
        g_running = 0;
        printf("CMD:STOPPED\n");
    }
    else if (strcmp(line, "STATUS") == 0) {
        int16_t enc_b, enc_c;
        pcnt_get_counter_value(motor_base.pcnt_unit, &enc_b);
        pcnt_get_counter_value(motor_codo.pcnt_unit, &enc_c);
        printf("STATUS:%s:%d:%d:%.2f:%.2f\n",
               g_running ? "RUNNING" : "IDLE",
               enc_b, enc_c,
               enc_b * BASE_RAD_PP,
               enc_c * CODO_RAD_PP);
    }
    // ---- VERSION: identificación del firmware ----
    else if (strcmp(line, "VERSION") == 0) {
        printf("VERSION:CALIB:3.0:NVS+WiFi\n");
    }
    // ---- NVS: guardar posición Home actual ----
    else if (strcmp(line, "SAVE_HOME") == 0) {
        int16_t e1, e2;
        pcnt_get_counter_value(motor_base.pcnt_unit, &e1);
        pcnt_get_counter_value(motor_codo.pcnt_unit, &e2);
        float th1 = e1 * BASE_RAD_PP;
        float th2 = e2 * CODO_RAD_PP;
        nvs_save_home(th1, th2);
        printf("HOME_SAVED:%.4f:%.4f\n", th1, th2);
    }
    // ---- NVS: cargar Home ----
    else if (strcmp(line, "LOAD_HOME") == 0) {
        float th1, th2;
        if (nvs_load_home(&th1, &th2))
            printf("HOME_LOADED:%.4f:%.4f\n", th1, th2);
        else
            printf("HOME_NOT_FOUND\n");
    }
    else if (strlen(line) > 0) {
        printf("CMD:UNKNOWN:%s\n", line);
    }
}

// ============================================================================
// TAREA DE MONITOREO DE CORRIENTE (protección principal por software)
// ============================================================================
// Lee la corriente de cada motor vía ADC cada 50ms.
// Si la corriente excede i_load_max, frena el motor inmediatamente.
// Es la ÚNICA protección activa (sin comparadores externos).

void current_monitor_task(void *pv) {
    ESP_LOGI(TAG, "Monitor de corriente iniciado (umbral base: %.1fA, codo: %.1fA)",
             motor_base.i_load_max, motor_codo.i_load_max);
    
    while (1) {
        if (g_running) {
            check_current_and_brake(&motor_base);
            check_current_and_brake(&motor_codo);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz
    }
}

// ============================================================================
// NVS - ALMACENAMIENTO PERSISTENTE (Home, límites, calibración)
// ============================================================================

void nvs_save_home(float th1, float th2) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_KEY_HOME_TH1, *(int32_t *)&th1);
        nvs_set_i32(h, NVS_KEY_HOME_TH2, *(int32_t *)&th2);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "✓ Home guardado en NVS: th1=%.2f° th2=%.2f°",
                 th1 * 180 / M_PI, th2 * 180 / M_PI);
    }
}

int nvs_load_home(float *th1, float *th2) {
    nvs_handle_t h;
    int32_t v1, v2;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        esp_err_t e1 = nvs_get_i32(h, NVS_KEY_HOME_TH1, &v1);
        esp_err_t e2 = nvs_get_i32(h, NVS_KEY_HOME_TH2, &v2);
        nvs_close(h);
        if (e1 == ESP_OK && e2 == ESP_OK) {
            *th1 = *(float *)&v1;
            *th2 = *(float *)&v2;
            ESP_LOGI(TAG, "✓ Home cargado de NVS: th1=%.2f° th2=%.2f°",
                     *th1 * 180 / M_PI, *th2 * 180 / M_PI);
            return 1;
        }
    }
    return 0;  // no hay datos guardados
}

void nvs_save_calib_done(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_CALIB_OK, 1);
        nvs_commit(h);
        nvs_close(h);
    }
}

int nvs_is_calib_done(void) {
    nvs_handle_t h;
    uint8_t val = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY_CALIB_OK, &val);
        nvs_close(h);
    }
    return val == 1;
}

// ============================================================================
// TAREA DE LOG FORZADO
// ============================================================================
// TAREA DE LOG FORZADO
// ============================================================================

void data_log_task(void *pv) {
    // Tarea de respaldo - actualmente el logging se hace inline en cada rutina
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ENTRY POINT (app_main)
// ============================================================================

void app_main(void) {
    // ---- Salida INMEDIATA para diagnóstico ----
    printf("\n\n=== ROBOT2R BOOT ===\n");
    ESP_LOGI(TAG, "Firmware v3.2 - MCPWM + NVS (solo USB)");

    // ---- 1. UART y tarea serial LO ANTES POSIBLE ----
    setbuf(stdin, NULL);
    setbuf(stdout, NULL);
    xTaskCreatePinnedToCore(serial_cmd_task, "serial_cmd", 4096, NULL, 6, NULL, 1);
    printf("READY\n");  // respuesta inmediata para discovery

    // ---- 2. NVS (persistencia) ----
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Borrando NVS corrupta...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS no disponible, continuando sin persistencia");
    }

    // ---- 3. Cargar Home desde NVS ----
    float home_th1, home_th2;
    if (nvs_load_home(&home_th1, &home_th2)) {
        ESP_LOGI(TAG, "Home restaurado: th1=%.2f° th2=%.2f°",
                 home_th1 * 180 / M_PI, home_th2 * 180 / M_PI);
    }
    if (nvs_is_calib_done()) {
        ESP_LOGI(TAG, "Calibracion previa en NVS.");
    }

    // ---- 4. ADC (corrientes) ----
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BASE_R_IS, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(BASE_L_IS, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(CODO_R_IS, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(CODO_L_IS, ADC_ATTEN_DB_12);

    // ---- 5. Motores (MCPWM + PCNT) ----
    ESP_LOGI(TAG, "Inicializando hardware de motores...");
    motor_init(&motor_base);
    ESP_LOGI(TAG, "Motor BASE OK");
    motor_init(&motor_codo);
    ESP_LOGI(TAG, "Motor CODO OK");
    ESP_LOGI(TAG, "Hardware listo.");

    // ---- 6. Tareas restantes ----
    data_queue = xQueueCreate(QUEUE_SIZE, sizeof(motor_sample_t));
    xTaskCreatePinnedToCore(current_monitor_task, "cur_mon", 2048, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(data_log_task, "data_log", 2048, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "Sistema listo. Comandos: SAVE_HOME, LOAD_HOME, STATUS, etc.");
    printf("\nREADY\n");
}