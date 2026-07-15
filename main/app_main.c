/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "foc/esp_foc.h"
#include "svpwm/esp_svpwm.h"

#include "filesystem.h"
#include "http-server.h"
#include "wifi.h"
#include ".wifi-secrets.h"
#include "as5600.h"


static const char *TAG = "example_foc";


// State
volatile float rpm = 10.0;
volatile float angle = 0.0;
volatile float encoder_angle = 0.0;
volatile bool motorEnable = true;

// For PID controller
// These worked well for a 1:1 gear ratio
//volatile float Kp = 1.5;
//volatile float Ki = 0.2;
//volatile float Kd = 0.02;

volatile float Kp = 1.5;
volatile float Ki = 0.2;
volatile float Kd = 0.02;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef MINI_S3
#pragma message("Configured for MINI_S3 board")
// Mini ESP32-S3 board used these
#define FOC_DRV1_EN_GPIO         10
#define FOC_DRV1_FAULT_GPIO      11
#define PITCH_FOC_PWM1_UH_GPIO          4
#define PITCH_FOC_PWM1_UL_GPIO          5
#define PITCH_FOC_PWM1_VH_GPIO          6
#define PITCH_FOC_PWM1_VL_GPIO          7
#define PITCH_FOC_PWM1_WH_GPIO          8
#define PITCH_FOC_PWM1_WL_GPIO          9
#else
#pragma message("Configured for ESP32 WROOM board")
// Larger EWSP32-S3 WROOM board uses these
// first controller
#define FOC_DRV1_EN_GPIO          2
#define FOC_DRV1_FAULT_GPIO       34
#define PITCH_FOC_PWM1_UH_GPIO          4
#define PITCH_FOC_PWM1_UL_GPIO          5
#define PITCH_FOC_PWM1_VH_GPIO          12
#define PITCH_FOC_PWM1_VL_GPIO          13
#define PITCH_FOC_PWM1_WH_GPIO          14
#define PITCH_FOC_PWM1_WL_GPIO          15

// second controller
#define FOC_DRV2_EN_GPIO          27
#define FOC_DRV2_FAULT_GPIO       35
#define YAW_FOC_PWM2_UH_GPIO       16
#define YAW_FOC_PWM2_UL_GPIO       17
#define YAW_FOC_PWM2_VH_GPIO       18
#define YAW_FOC_PWM2_VL_GPIO       19
#define YAW_FOC_PWM2_WH_GPIO       25
#define YAW_FOC_PWM2_WL_GPIO       26

#endif

#define FOC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define FOC_MCPWM_PERIOD              1000     // 1000 * 0.1us = 100us, 10KHz
#define FOC_LOOP_DT_S (1.0f / (FOC_MCPWM_TIMER_RESOLUTION_HZ / (float)FOC_MCPWM_PERIOD))

// Position PID output feeds torque_offset, the electrical lead angle ahead of the
// rotor's electrical angle. Torque falls off past 90 deg lead, so clamp well inside that.
#define MAX_TORQUE_OFFSET_DEG 80.0f

// AS5600 is polled from its own task (see as5600_task) rather than inline in the
// 10kHz FOC loop -- a single I2C transaction can easily take longer than the 100us
// PWM period, which was stalling/jittering commutation. 2ms is comfortably slower
// than one transaction takes, and far faster than this mechanism's motion.
#define AS5600_POLL_PERIOD_MS 2

float foc_wave_freq = 50.0;         // 50Hz 3 phase AC wave
#define EXAMPLE_FOC_WAVE_AMPL    100  // Wave amplitude, Use up-down timer mode, max value should be (FOC_MCPWM_PERIOD/2)
#define POLE_PAIRS 7

// Motor shaft revolutions per one revolution of the shaft the AS5600 actually
// tracks. 1.0 means the sensor sits directly on the motor shaft (old
// assumption). Set this to the real reduction ratio when the sensor is on a
// slower, geared-down output/aim shaft.
// 20-tooth motor gear driving a 100-tooth output gear -> motor turns 5x per
// output/sensor revolution.
#define GEAR_RATIO 5.0f

void bsp_bridge_driver_init(void)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << FOC_DRV1_EN_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
}

void bsp_bridge_driver_enable(bool enable)
{
    ESP_LOGI(TAG, "%s MOSFET gate", enable ? "Enable" : "Disable");
    gpio_set_level(FOC_DRV1_EN_GPIO, enable);
}

bool inverter_update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t task_yield = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);
    return task_yield;
}

inverter_handle_t inverter1;
foc_dq_coord_t dq_out = {_IQ(0), _IQ(0)};

// Shortest signed error from current to target, wrapped to (-180, 180] degrees.
float wrap_angle_error_deg(float target_deg, float current_deg) {
    float diff = fmodf(target_deg - current_deg, 360.0f);
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff <= -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

void foc_set_angle(float elec_theta_deg) {
	foc_ab_coord_t ab_out;
    foc_uvw_coord_t uvw_out;
	int uvw_duty[3];
	_iq elec_theta_rad = _IQmpy(_IQ(elec_theta_deg), _IQ(M_PI / 180.f));
    // In FOC motor control, we usually set Vd for alignment or weak-meg control, and set Vq for torque control.
    // As here is open loop output, use Vd is enough, and coord aligned
    dq_out.d = _IQ(EXAMPLE_FOC_WAVE_AMPL);
    foc_inverse_park_transform(elec_theta_rad, &dq_out, &ab_out);

    #if CONFIG_ESP_FOC_USE_SVPWM
        foc_svpwm_duty_calculate(&ab_out, &uvw_out);
    #else   // Use spwm (sin pwm) instead. (see menuconfig help to know difference between SVPWM and SPWM)
        foc_inverse_clarke_transform(&ab_out, &uvw_out);
    #endif
    // Regular uvw data to (0 ~ (FOC_MCPWM_PERIOD/2))
    uvw_duty[0] = _IQtoF(_IQdiv2(uvw_out.u)) + (FOC_MCPWM_PERIOD / 4);
    uvw_duty[1] = _IQtoF(_IQdiv2(uvw_out.v)) + (FOC_MCPWM_PERIOD / 4);
    uvw_duty[2] = _IQtoF(_IQdiv2(uvw_out.w)) + (FOC_MCPWM_PERIOD / 4);

    // output pwm duty
    ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter1, uvw_duty[0], uvw_duty[1], uvw_duty[2]));
}

// Rate of change of encoder_angle, in deg/s, computed by as5600_task from its own
// measured sample interval. Internal to this file -- foc_task uses it in place of
// differentiating encoder_angle itself against the FOC loop's fixed tick rate.
static volatile float encoder_velocity_deg_s = 0.0f;

// Polls the AS5600 on its own schedule and publishes encoder_angle/encoder_velocity_deg_s.
// Pinned to core 0 (with WiFi/HTTP) so its I2C transactions never block the
// time-critical FOC loop on core 1 -- foc_task must never touch the sensor directly.
void as5600_task(void *arg)
{
    as5600_init(0);

    float last_deg = as5600_get_mech_deg(0);
    int64_t last_us = esp_timer_get_time();
    encoder_angle = last_deg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(AS5600_POLL_PERIOD_MS));

        float deg = as5600_get_mech_deg(0);
        int64_t now_us = esp_timer_get_time();
        float dt_s = (now_us - last_us) / 1e6f;

        encoder_velocity_deg_s = wrap_angle_error_deg(deg, last_deg) / dt_s;
        encoder_angle = deg;

        last_deg = deg;
        last_us = now_us;
    }
}

// Runs the inverter/PWM setup, sensor calibration, and the FOC control loop.
// Pinned to core 1 so it never contends with WiFi/HTTP handling on core 0.
void foc_task(void *arg)
{
    // counting semaphore used to sync update foc calculation when mcpwm timer updated
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    float elec_theta_deg = 0;

    inverter_config_t cfg = {
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,   //UP_DOWN mode will generate center align pwm wave, which can reduce MOSFET switch times on same effect, extend life
            .period_ticks = FOC_MCPWM_PERIOD,
        },
        .operator_config = {
            .group_id = 0,
        },
        .compare_config = {
            .flags.update_cmp_on_tez = true,
        },
        .gen_gpios = {
            {PITCH_FOC_PWM1_UH_GPIO, PITCH_FOC_PWM1_UL_GPIO},
            {PITCH_FOC_PWM1_VH_GPIO, PITCH_FOC_PWM1_VL_GPIO},
            {PITCH_FOC_PWM1_WH_GPIO, PITCH_FOC_PWM1_WL_GPIO},
        },
        .dt_config = {
            .posedge_delay_ticks = 5,
        },
        .inv_dt_config = {
            .negedge_delay_ticks = 5,
            .flags.invert_output = true,
        },
    };
    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter1));
    ESP_LOGI(TAG, "Inverter init OK");

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = inverter_update_cb,
    };
    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter1, &cbs, &update_semaphore));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Inverter start OK");

    // Enable gate driver chip
    bsp_bridge_driver_init();
    bsp_bridge_driver_enable(true);

    // Sensor is owned by as5600_task on core 0; this just logs whatever it has
    // published so far. as5600_task is started well before this point in
    // app_main, so a real reading is expected here.
    ESP_LOGI(TAG, "Mechanical angle: %.2f deg", encoder_angle);

    // force 0 degree electrical angle
    // after inverter enable
    float align_angle = 0.0f;
    for (int i = 0; i < 500; i++) {
        foc_set_angle(align_angle);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Continuous (unwrapped) tracking of the AS5600's own shaft angle, so
    // commutation can see the motor turn through many full revolutions
    // instead of losing position every time the sensor wraps past 360/0.
    // Seeded from the alignment reading, so it starts at 0 relative to the
    // electrical zero set above.
    float last_raw_mech_deg = encoder_angle;
    float mech_deg_gear_accum = 0.0f;
    ESP_LOGI(TAG, "Mech. zero: %.2f deg", last_raw_mech_deg);

    // Aim position PID state (persists across loop iterations)
    float aim_error_integral = 0.0f;

    ESP_LOGI(TAG, "Start FOC");
    while (true) {
        xSemaphoreTake(update_semaphore, portMAX_DELAY);

        // electrical Hz is foc_wave_freq
        // float mechanical_Hz = electrical_Hz / POLE_PAIRS;
        // float mechanical_RPM = (60 × electrical_Hz) / POLE_PAIRS;

#if false
        // this is simple open-loop calculation
        foc_wave_freq = rpm * POLE_PAIRS / 60.0;
        // Calculate elec_theta_deg increase step of 50Hz output on 10000Hz call
        elec_theta_deg += (foc_wave_freq * 360.f) / (FOC_MCPWM_TIMER_RESOLUTION_HZ / FOC_MCPWM_PERIOD);
#else
        // this is closed-loop calculation using encoder feedback.
        // raw_mech_deg/encoder_velocity_deg_s come from as5600_task (core 0) --
        // this loop never talks to the sensor directly, so it actually holds the
        // hardware-clocked 100us MCPWM tick instead of stalling on I2C.
        float raw_mech_deg = encoder_angle;

        // Fold the latest sensor step into the unwrapped accumulator using the
        // shortest signed delta, so a wrap past 360/0 doesn't register as a
        // huge jump. Safe as long as the sensor can't move >180 gear-deg
        // between FOC loop iterations, which at a 100us tick it never does.
        mech_deg_gear_accum += wrap_angle_error_deg(raw_mech_deg, last_raw_mech_deg);
        last_raw_mech_deg = raw_mech_deg;

        // GEAR_RATIO reconstructs the motor shaft's own (continuous,
        // multi-turn) angle from the geared-down sensor reading, so
        // commutation tracks the real rotor instead of stalling after one
        // sensor count's worth of motion.
        float mech_deg_motor = mech_deg_gear_accum * GEAR_RATIO;
        float rotor_elec = - mech_deg_motor * POLE_PAIRS;

        // ---- Aim position control ----
        // PID on the shortest angular error drives torque_offset (electrical lead
        // angle), which steers the aim angle toward the web UI's commanded `angle`.
        // `angle` and raw_mech_deg both live in the AS5600's own (single-turn,
        // wrapped) frame, so this shortest-path error is correct regardless of
        // GEAR_RATIO -- it's only the commutation angle above that needs the
        // motor-shaft reconstruction.
        float aim_error_deg = wrap_angle_error_deg(angle, raw_mech_deg);
        aim_error_integral += aim_error_deg * FOC_LOOP_DT_S;
        aim_error_integral = fmaxf(fminf(aim_error_integral, MAX_TORQUE_OFFSET_DEG), -MAX_TORQUE_OFFSET_DEG);

        // Error derivative ~= -position velocity (assumes `angle`, the human-set aim
        // target, changes slowly compared to the encoder's sampling rate). Using
        // as5600_task's own measured velocity avoids differentiating a signal that
        // only actually updates every ~2ms against this loop's fixed 100us tick --
        // that mismatch was amplifying encoder noise into large torque swings.
        float aim_error_deriv = -encoder_velocity_deg_s;

        // Negated: with rotor_elec now using -mech_deg (see sign fix above), a
        // positive torque_offset physically drives raw_mech_deg DOWN. Without this
        // negation the loop pushes away from the target instead of toward it --
        // which, thanks to the wrapped error metric, doesn't run away to infinity
        // but settles into a stable equilibrium at the antipodal point (180 deg
        // off target), matching the "buzzes at 180 deg out of phase" symptom.
        float torque_offset = -(Kp * aim_error_deg + Ki * aim_error_integral + Kd * aim_error_deriv);
        torque_offset = fmaxf(fminf(torque_offset, MAX_TORQUE_OFFSET_DEG), -MAX_TORQUE_OFFSET_DEG);

        elec_theta_deg = rotor_elec + torque_offset;
#endif
        // rotor_elec can be many multiples of 360 once the motor has turned
        // several revolutions (GEAR_RATIO > 1 especially), so wrap with
        // fmodf rather than a single conditional subtract/add.
        elec_theta_deg = fmodf(elec_theta_deg, 360.0f);
        if (elec_theta_deg < 0) {
            elec_theta_deg += 360.0f;
        }

        foc_set_angle(elec_theta_deg);
    }

    bsp_bridge_driver_enable(false);
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC");

    esp_err_t ret = start_filesystem();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start filesystem: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Files in root directory:");
    list_files("/fs");
    ESP_LOGI(TAG, "Files in /fs/www directory:");
    list_files("/fs/www");

    // Sensor polling stays on core 0 too, alongside WiFi/HTTP -- keeps I2C entirely
    // off the time-critical FOC loop on core 1. Started early so it has a real
    // reading published well before foc_task's alignment step needs one.
    xTaskCreatePinnedToCore(as5600_task, "as5600_task", 4096, NULL, 5, NULL, 0);

    // WiFi event handling (esp_event/wifi driver tasks) stays on core 0, per
    // CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0 and CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0.
    wifi_init();
    wifi_scan_and_print();
    ESP_LOGI(TAG, "Connecting to WiFi...");
    wifi_connect(AP_NAME, AP_PASSWORD);

    // FOC computation and PWM adjustment run in their own task, pinned to core 1.
    xTaskCreatePinnedToCore(foc_task, "foc_task", 4096, NULL, 5, NULL, 1);
}
