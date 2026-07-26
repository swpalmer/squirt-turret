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
// Commanded elevation for the second (pitch) DRV8302 axis, PITCH_MIN_DEG to
// PITCH_MAX_DEG (horizon-front to horizon-back, through straight up).
volatile float pitch = 0.0;
// Capstan-corrected, unwrapped actual pitch angle -- see pitch_task.
volatile float pitch_actual = 0.0;
// Pitch's own thermal-idle gate flag (separate driver from azimuth's motorEnable).
volatile bool pitchMotorEnable = true;
// True once pitch_task has homed against the limit switch; pitch commands are
// ignored until then.
volatile bool pitchCalibrated = false;
// True if homing hit its safety travel cutoff without finding the limit
// switch -- pitch stays disabled rather than jogging indefinitely.
volatile bool pitchHomingFailed = false;

// For PID controller
// These worked well for a 1:1 gear ratio
//volatile float Kp = 1.5;
//volatile float Ki = 0.2;
//volatile float Kd = 0.02;

volatile float Kp = 2.5;
volatile float Ki = 2.0;
volatile float Kd = 0.2;

// Pitch PID gains -- starting point copied from azimuth's, needs its own
// tuning pass once the capstan/limit-switch hardware is in place.
volatile float PitchKp = 2.5;
volatile float PitchKi = 2.0;
volatile float PitchKd = 0.2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec /////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef MINI_S3
#pragma message("Configured for MINI_S3 board")
// Mini ESP32-S3 board used these
#define FOC_DRV1_EN_GPIO         10
#define FOC_DRV1_FAULT_GPIO      11
#define AZIMUTH_FOC_PWM1_UH_GPIO          4
#define AZIMUTH_FOC_PWM1_UL_GPIO          5
#define AZIMUTH_FOC_PWM1_VH_GPIO          6
#define AZIMUTH_FOC_PWM1_VL_GPIO          7
#define AZIMUTH_FOC_PWM1_WH_GPIO          8
#define AZIMUTH_FOC_PWM1_WL_GPIO          9
// No second driver/pitch axis wired up for this board yet.
#else
#pragma message("Configured for ESP32 WROOM board")
// Larger EWSP32-S3 WROOM board uses these
// first controller (azimuth)
#define FOC_DRV1_EN_GPIO          2
#define FOC_DRV1_FAULT_GPIO       34
#define AZIMUTH_FOC_PWM1_UH_GPIO          4
#define AZIMUTH_FOC_PWM1_UL_GPIO          5
#define AZIMUTH_FOC_PWM1_VH_GPIO          12
#define AZIMUTH_FOC_PWM1_VL_GPIO          13
#define AZIMUTH_FOC_PWM1_WH_GPIO          14
#define AZIMUTH_FOC_PWM1_WL_GPIO          15

// second controller (pitch)
#define FOC_DRV2_EN_GPIO          27
#define FOC_DRV2_FAULT_GPIO       35
#define PITCH_FOC_PWM2_UH_GPIO       16
#define PITCH_FOC_PWM2_UL_GPIO       17
#define PITCH_FOC_PWM2_VH_GPIO       18
#define PITCH_FOC_PWM2_VL_GPIO       19
#define PITCH_FOC_PWM2_WH_GPIO       25
#define PITCH_FOC_PWM2_WL_GPIO       26

// TODO: not wired yet -- placeholder. Update GPIO (and active level in
// pitch_task's gpio_config_t, if not active-low) once the switch is actually
// installed.
#define PITCH_LIMIT_SWITCH_GPIO   39

#endif

#define FOC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define FOC_MCPWM_PERIOD              1000     // 1000 * 0.1us = 100us, 10KHz
#define FOC_LOOP_DT_S (1.0f / (FOC_MCPWM_TIMER_RESOLUTION_HZ / (float)FOC_MCPWM_PERIOD))

// Position PID output feeds torque_offset, the electrical lead angle ahead of the
// rotor's electrical angle. Torque falls off past 90 deg lead, so clamp well inside that.
#define MAX_TORQUE_OFFSET_DEG 88.0f

// This project drives a fixed current amplitude (DRV1_FOC_WAVE_AMPL) regardless
// of how much torque is actually needed, so holding position burns just as much
// current -- and generates just as much heat -- as actively correcting a large
// error. Once the turret is within DRIVE_DISABLE_ERROR_DEG of its target, gate the
// DRV8302 off entirely rather than idle-holding. The wider DRIVE_ENABLE_ERROR_DEG
// re-enable threshold (hysteresis) keeps the gate from chattering right at the
// disable boundary.
#define DRIVE_DISABLE_ERROR_DEG 1.5f
#define DRIVE_ENABLE_ERROR_DEG  4.0f

// AS5600 is polled from its own task (see as5600_task) rather than inline in the
// 10kHz FOC loop -- a single I2C transaction can easily take longer than the 100us
// PWM period, which was stalling/jittering commutation. 2ms is comfortably slower
// than one transaction takes, and far faster than this mechanism's motion.
#define AS5600_POLL_PERIOD_MS 2

float foc_wave_freq = 50.0;         // 50Hz 3 phase AC wave
#define DRV1_FOC_WAVE_AMPL    150  // Wave amplitude, Use up-down timer mode, max value should be (FOC_MCPWM_PERIOD/2)
// Floor amplitude used when torque_offset is ~0 (settled, or barely
// correcting). The d-axis voltage drives winding current regardless of lead
// angle, so holding at full DRV1_FOC_WAVE_AMPL burns full current for
// whatever torque is actually needed -- scaling amplitude down with
// |torque_offset| (see foc_task) cuts that waste for small corrections while
// still ramping back up to full amplitude for large moves.
#define MIN_FOC_WAVE_AMPL     40
#define POLE_PAIRS 7

// Motor shaft revolutions per one revolution of the shaft the AS5600 actually
// tracks. 1.0 means the sensor sits directly on the motor shaft (old
// assumption). Set this to the real reduction ratio when the sensor is on a
// slower, geared-down output/aim shaft.
// 20-tooth motor gear driving a 100-tooth output gear -> motor turns 5x per
// output/sensor revolution.
#define GEAR_RATIO 5.0f

// ---- Pitch axis (capstan drive) ----
// Unlike azimuth, the pitch AS5600 sits directly on the motor shaft, and a
// capstan (15mm motor-side pulley, 150mm output-side pulley) reduces motor
// rotation down to the output/pitch shaft -- the inverse arrangement from
// GEAR_RATIO above. Motor revs per one output/pitch revolution:
#define CAPSTAN_RATIO 10.0f

// Commandable pitch range. True physical range is somewhere between 180 and
// 300 deg (the gun can pitch back past horizontal); start conservative and
// widen once the real travel is measured against the limit switch.
#define PITCH_MIN_DEG 0.0f
#define PITCH_MAX_DEG 180.0f
// Pitch value assigned to the position where the limit switch trips.
#define PITCH_HOME_DEG PITCH_MIN_DEG

// Homing: jog slowly toward the limit switch (at the pitch minimum) at low
// current until it trips. PITCH_HOMING_DIRECTION is the sign of that jog in
// electrical degrees -- flip it if homing is observed driving away from the
// switch instead of toward it.
#define PITCH_HOMING_DIRECTION -1.0f
#define PITCH_HOMING_STEP_DEG_PER_TICK 0.5f
#define PITCH_HOMING_WAVE_AMPL MIN_FOC_WAVE_AMPL
// Safety cutoff for homing, independent of PITCH_MAX_DEG -- sized to the
// worst-case ~300 deg physical travel (not the narrower commandable range),
// so homing isn't declared failed before it could have reached a properly
// installed switch. Expected to trip on every boot until the switch is
// actually wired, since PITCH_LIMIT_SWITCH_GPIO is currently a placeholder.
#define PITCH_HOMING_MAX_TRAVEL_DEG (320.0f * CAPSTAN_RATIO)

void bsp_bridge_driver_init(int en_gpio)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << en_gpio,
    };
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
}

void bsp_bridge_driver_enable(int en_gpio, bool enable)
{
    ESP_LOGI(TAG, "%s MOSFET gate (gpio %d)", enable ? "Enable" : "Disable", en_gpio);
    gpio_set_level(en_gpio, enable);
}

bool inverter_update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{
    BaseType_t task_yield = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);
    return task_yield;
}

inverter_handle_t inverter1;
#ifndef MINI_S3
inverter_handle_t inverter2;
#endif

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

// dq_out is local (not shared) so this is safe to call concurrently from both
// foc_task and pitch_task on their own inverters.
void foc_set_angle(inverter_handle_t inverter, float elec_theta_deg, float wave_ampl) {
	foc_ab_coord_t ab_out;
    foc_uvw_coord_t uvw_out;
	int uvw_duty[3];
	foc_dq_coord_t dq_out = {_IQ(0), _IQ(0)};
	_iq elec_theta_rad = _IQmpy(_IQ(elec_theta_deg), _IQ(M_PI / 180.f));
    // In FOC motor control, we usually set Vd for alignment or weak-meg control, and set Vq for torque control.
    // As here is open loop output, use Vd is enough, and coord aligned
    dq_out.d = _IQ(wave_ampl);
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
    ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter, uvw_duty[0], uvw_duty[1], uvw_duty[2]));
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

#ifndef MINI_S3
// Pitch's AS5600 lives on axis 1 (its own I2C bus, see as5600.c) and sits
// directly on the motor shaft rather than a geared-down output shaft, so
// these are motor-frame, single-turn readings -- pitch_task converts them
// into the capstan-corrected, unwrapped pitch_actual.
volatile float pitch_encoder_raw_deg = 0.0f;
static volatile float pitch_encoder_velocity_deg_s = 0.0f;

// Same role as as5600_task, for the pitch axis. Pinned to core 0 alongside it.
void pitch_as5600_task(void *arg)
{
    as5600_init(1);

    float last_deg = as5600_get_mech_deg(1);
    int64_t last_us = esp_timer_get_time();
    pitch_encoder_raw_deg = last_deg;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(AS5600_POLL_PERIOD_MS));

        float deg = as5600_get_mech_deg(1);
        int64_t now_us = esp_timer_get_time();
        float dt_s = (now_us - last_us) / 1e6f;

        pitch_encoder_velocity_deg_s = wrap_angle_error_deg(deg, last_deg) / dt_s;
        pitch_encoder_raw_deg = deg;

        last_deg = deg;
        last_us = now_us;
    }
}
#endif

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
            {AZIMUTH_FOC_PWM1_UH_GPIO, AZIMUTH_FOC_PWM1_UL_GPIO},
            {AZIMUTH_FOC_PWM1_VH_GPIO, AZIMUTH_FOC_PWM1_VL_GPIO},
            {AZIMUTH_FOC_PWM1_WH_GPIO, AZIMUTH_FOC_PWM1_WL_GPIO},
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
    bsp_bridge_driver_init(FOC_DRV1_EN_GPIO);
    bsp_bridge_driver_enable(FOC_DRV1_EN_GPIO, true);

    // Sensor is owned by as5600_task on core 0; this just logs whatever it has
    // published so far. as5600_task is started well before this point in
    // app_main, so a real reading is expected here.
    ESP_LOGI(TAG, "Mechanical angle: %.2f deg", encoder_angle);

    // force 0 degree electrical angle
    // after inverter enable
    float align_angle = 0.0f;
    for (int i = 0; i < 500; i++) {
        foc_set_angle(inverter1, align_angle, DRV1_FOC_WAVE_AMPL);
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

        // `angle` and raw_mech_deg both live in the AS5600's own (single-turn,
        // wrapped) frame, so this shortest-path error is correct regardless of
        // GEAR_RATIO -- it's only the commutation angle above that needs the
        // motor-shaft reconstruction.
        float aim_error_deg = wrap_angle_error_deg(angle, raw_mech_deg);

        // ---- Thermal idle ----
        // Gate the driver off once settled, back on once the target has moved
        // away again. motorEnable also doubles as the web UI's driver-state
        // readout (see http-server.c).
        if (motorEnable) {
            if (fabsf(aim_error_deg) < DRIVE_DISABLE_ERROR_DEG) {
                motorEnable = false;
                aim_error_integral = 0.0f;
                bsp_bridge_driver_enable(FOC_DRV1_EN_GPIO, false);
            }
        } else if (fabsf(aim_error_deg) > DRIVE_ENABLE_ERROR_DEG) {
            motorEnable = true;
            bsp_bridge_driver_enable(FOC_DRV1_EN_GPIO, true);
        }

        if (!motorEnable) {
            // Driver is gated off -- nothing to commutate. mech_deg_gear_accum
            // above keeps tracking the sensor regardless, so rotor_elec is
            // still correct the instant the driver comes back on.
            continue;
        }

        // ---- Aim position control ----
        // PID on the shortest angular error drives torque_offset (electrical lead
        // angle), which steers the aim angle toward the web UI's commanded `angle`.
        aim_error_integral += aim_error_deg * FOC_LOOP_DT_S;
        // Anti-windup: bound the accumulator so Ki * aim_error_integral itself
        // can't exceed the torque ceiling, whatever Ki is set to. Clamping the
        // raw accumulator directly to MAX_TORQUE_OFFSET_DEG (as before) capped
        // the integral's real contribution at Ki * MAX_TORQUE_OFFSET_DEG -- for
        // Ki < 1 that's well short of the actual ceiling, so the integral could
        // never build up enough lead angle to break static friction/cogging
        // near the target no matter how long the error persisted.
        if (Ki > 0.0f) {
            float integral_limit = MAX_TORQUE_OFFSET_DEG / Ki;
            aim_error_integral = fmaxf(fminf(aim_error_integral, integral_limit), -integral_limit);
        }

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

        // Scale d-axis voltage with how much lead angle is actually being
        // commanded, so small corrections/near-target holding draw well under
        // full current instead of the fixed DRV1_FOC_WAVE_AMPL regardless of
        // load -- see MIN_FOC_WAVE_AMPL comment above.
        float wave_ampl = MIN_FOC_WAVE_AMPL + (DRV1_FOC_WAVE_AMPL - MIN_FOC_WAVE_AMPL)
            * (fabsf(torque_offset) / MAX_TORQUE_OFFSET_DEG);
#endif
        // rotor_elec can be many multiples of 360 once the motor has turned
        // several revolutions (GEAR_RATIO > 1 especially), so wrap with
        // fmodf rather than a single conditional subtract/add.
        elec_theta_deg = fmodf(elec_theta_deg, 360.0f);
        if (elec_theta_deg < 0) {
            elec_theta_deg += 360.0f;
        }

        foc_set_angle(inverter1, elec_theta_deg, wave_ampl);
    }

    bsp_bridge_driver_enable(FOC_DRV1_EN_GPIO, false);
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
    vTaskDelete(NULL);
}

#ifndef MINI_S3
// Homes against the limit switch (pitch minimum), then runs closed-loop PID
// position control on the capstan-corrected pitch_actual. Mirrors foc_task's
// structure/tuning approach; differs in that the AS5600 is on the motor shaft
// (not a geared-down output shaft), so CAPSTAN_RATIO divides where foc_task's
// GEAR_RATIO multiplies, and position error is a plain (unwrapped) subtraction
// since pitch is a bounded linear range, not a full rotation.
void pitch_task(void *arg)
{
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    float elec_theta_deg = 0;

    inverter_config_t cfg = {
        .timer_config = {
            .group_id = 1,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
            .period_ticks = FOC_MCPWM_PERIOD,
        },
        .operator_config = {
            .group_id = 1,
        },
        .compare_config = {
            .flags.update_cmp_on_tez = true,
        },
        .gen_gpios = {
            {PITCH_FOC_PWM2_UH_GPIO, PITCH_FOC_PWM2_UL_GPIO},
            {PITCH_FOC_PWM2_VH_GPIO, PITCH_FOC_PWM2_VL_GPIO},
            {PITCH_FOC_PWM2_WH_GPIO, PITCH_FOC_PWM2_WL_GPIO},
        },
        .dt_config = {
            .posedge_delay_ticks = 5,
        },
        .inv_dt_config = {
            .negedge_delay_ticks = 5,
            .flags.invert_output = true,
        },
    };
    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter2));
    ESP_LOGI(TAG, "Pitch inverter init OK");

    mcpwm_timer_event_callbacks_t cbs = {
        .on_full = inverter_update_cb,
    };
    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter2, &cbs, &update_semaphore));
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter2, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Pitch inverter start OK");

    bsp_bridge_driver_init(FOC_DRV2_EN_GPIO);
    bsp_bridge_driver_enable(FOC_DRV2_EN_GPIO, true);

    // TODO: placeholder pin -- see PITCH_LIMIT_SWITCH_GPIO definition. Assumes
    // active-low (switch pulls to GND); flip the level check below if wired
    // active-high instead.
    gpio_config_t limit_switch_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << PITCH_LIMIT_SWITCH_GPIO,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limit_switch_config));

    ESP_LOGI(TAG, "Pitch mechanical angle: %.2f deg", pitch_encoder_raw_deg);

    // force 0 degree electrical angle after inverter enable, same as foc_task
    float align_angle = 0.0f;
    for (int i = 0; i < 500; i++) {
        foc_set_angle(inverter2, align_angle, DRV1_FOC_WAVE_AMPL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Unwrapped motor-shaft angle, same accumulation technique as foc_task's
    // mech_deg_gear_accum.
    float last_raw_mech_deg = pitch_encoder_raw_deg;
    float mech_deg_gear_accum = 0.0f;
    // mech_deg_gear_accum's value at the instant the limit switch trips --
    // the zero point pitch_actual is measured from.
    float home_accum_offset = 0.0f;

    ESP_LOGI(TAG, "Homing pitch...");
    while (!pitchCalibrated && !pitchHomingFailed) {
        float raw_mech_deg = pitch_encoder_raw_deg;
        mech_deg_gear_accum += wrap_angle_error_deg(raw_mech_deg, last_raw_mech_deg);
        last_raw_mech_deg = raw_mech_deg;

        bool limit_switch_triggered = (gpio_get_level(PITCH_LIMIT_SWITCH_GPIO) == 0);
        if (limit_switch_triggered) {
            home_accum_offset = mech_deg_gear_accum;
            pitch_actual = PITCH_HOME_DEG;
            pitchCalibrated = true;
            ESP_LOGI(TAG, "Pitch homed at motor accum %.2f deg", home_accum_offset);
            break;
        }

        if (fabsf(mech_deg_gear_accum) > PITCH_HOMING_MAX_TRAVEL_DEG) {
            // Fails safe: stays disabled below rather than jogging forever.
            // Expected until PITCH_LIMIT_SWITCH_GPIO is wired to a real switch.
            pitchHomingFailed = true;
            bsp_bridge_driver_enable(FOC_DRV2_EN_GPIO, false);
            ESP_LOGE(TAG, "Pitch homing failed: travelled %.2f deg without tripping the limit switch", mech_deg_gear_accum);
            break;
        }

        align_angle = fmodf(align_angle + PITCH_HOMING_DIRECTION * PITCH_HOMING_STEP_DEG_PER_TICK, 360.0f);
        if (align_angle < 0) {
            align_angle += 360.0f;
        }
        foc_set_angle(inverter2, align_angle, PITCH_HOMING_WAVE_AMPL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (pitchHomingFailed) {
        vTaskDelete(NULL);
        return;
    }

    // Pitch position PID state (persists across loop iterations)
    float pitch_error_integral = 0.0f;

    ESP_LOGI(TAG, "Start pitch FOC");
    while (true) {
        xSemaphoreTake(update_semaphore, portMAX_DELAY);

        float raw_mech_deg = pitch_encoder_raw_deg;
        mech_deg_gear_accum += wrap_angle_error_deg(raw_mech_deg, last_raw_mech_deg);
        last_raw_mech_deg = raw_mech_deg;

        // Sensor is already on the motor shaft (ratio 1:1) -- unlike
        // foc_task, no GEAR_RATIO multiply is needed for commutation.
        float rotor_elec = -mech_deg_gear_accum * POLE_PAIRS;

        // Capstan-corrected, unwrapped output angle -- CAPSTAN_RATIO divides
        // (motor spins faster than the output), the inverse of GEAR_RATIO's
        // multiply in foc_task.
        pitch_actual = PITCH_HOME_DEG + (mech_deg_gear_accum - home_accum_offset) / CAPSTAN_RATIO;

        // Pitch is a bounded linear range, not a rotation, so this is a plain
        // (unwrapped) error -- no wrap_angle_error_deg needed.
        float pitch_target = fmaxf(fminf(pitch, PITCH_MAX_DEG), PITCH_MIN_DEG);
        float pitch_error_deg = pitch_target - pitch_actual;

        // ---- Thermal idle ---- (mirrors foc_task's, own enable flag/gpio)
        if (pitchMotorEnable) {
            if (fabsf(pitch_error_deg) < DRIVE_DISABLE_ERROR_DEG) {
                pitchMotorEnable = false;
                pitch_error_integral = 0.0f;
                bsp_bridge_driver_enable(FOC_DRV2_EN_GPIO, false);
            }
        } else if (fabsf(pitch_error_deg) > DRIVE_ENABLE_ERROR_DEG) {
            pitchMotorEnable = true;
            bsp_bridge_driver_enable(FOC_DRV2_EN_GPIO, true);
        }

        if (!pitchMotorEnable) {
            continue;
        }

        // ---- Pitch position control ---- (same PID/anti-windup shape as foc_task)
        pitch_error_integral += pitch_error_deg * FOC_LOOP_DT_S;
        if (PitchKi > 0.0f) {
            float integral_limit = MAX_TORQUE_OFFSET_DEG / PitchKi;
            pitch_error_integral = fmaxf(fminf(pitch_error_integral, integral_limit), -integral_limit);
        }

        // Convert the motor-frame sensor velocity into output/pitch-frame
        // velocity to match pitch_error_deg's units.
        float pitch_error_deriv = -pitch_encoder_velocity_deg_s / CAPSTAN_RATIO;

        // Negated for the same reason as foc_task's torque_offset: rotor_elec
        // uses -mech_deg_gear_accum, so a positive torque_offset drives
        // raw_mech_deg DOWN.
        float torque_offset = -(PitchKp * pitch_error_deg + PitchKi * pitch_error_integral + PitchKd * pitch_error_deriv);
        torque_offset = fmaxf(fminf(torque_offset, MAX_TORQUE_OFFSET_DEG), -MAX_TORQUE_OFFSET_DEG);

        elec_theta_deg = rotor_elec + torque_offset;

        float wave_ampl = MIN_FOC_WAVE_AMPL + (DRV1_FOC_WAVE_AMPL - MIN_FOC_WAVE_AMPL)
            * (fabsf(torque_offset) / MAX_TORQUE_OFFSET_DEG);

        elec_theta_deg = fmodf(elec_theta_deg, 360.0f);
        if (elec_theta_deg < 0) {
            elec_theta_deg += 360.0f;
        }

        foc_set_angle(inverter2, elec_theta_deg, wave_ampl);
    }

    bsp_bridge_driver_enable(FOC_DRV2_EN_GPIO, false);
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter2, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(svpwm_del_inverter(inverter2));
    vTaskDelete(NULL);
}
#endif

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
#ifndef MINI_S3
    xTaskCreatePinnedToCore(pitch_as5600_task, "pitch_as5600_task", 4096, NULL, 5, NULL, 0);
#endif

    // WiFi event handling (esp_event/wifi driver tasks) stays on core 0, per
    // CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0 and CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0.
    wifi_init();
    wifi_scan_and_print();
    ESP_LOGI(TAG, "Connecting to WiFi...");
    wifi_connect(AP_NAME, AP_PASSWORD);

    // FOC computation and PWM adjustment run in their own task, pinned to core 1.
    xTaskCreatePinnedToCore(foc_task, "foc_task", 4096, NULL, 5, NULL, 1);
#ifndef MINI_S3
    // Pitch's own FOC task/inverter (group 1), also pinned to core 1 so
    // homing/commutation never contends with WiFi/HTTP timing either.
    xTaskCreatePinnedToCore(pitch_task, "pitch_task", 4096, NULL, 5, NULL, 1);
#endif
}
