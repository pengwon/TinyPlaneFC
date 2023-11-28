#include "motor.h"
#include "pid.h"
#include "esp_log.h"

extern adc_oneshot_unit_handle_t adc1_handle;

static int current_offset_l = 0;
static int current_offset_r = 0;
static delta_pid_t pid;
static int duty_l = 0;
static int duty_r = 0;

void motor_init(void)
{
    // Prepare and then apply the PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = MOTOR_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = L_MOTOR_IO_NUM,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_1;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num       = R_MOTOR_IO_NUM;
    ledc_channel.duty           = 0; // Set duty to 0%
    ledc_channel.hpoint         = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void get_current(int *current_l, int *current_r)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, L_MOTOR_ADC_CHANNEL, current_l));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, R_MOTOR_ADC_CHANNEL, current_r));
}

void motor_update_current_offset(void)
{
    int current_l = 0;
    int current_r = 0;

    motor_set_duty(0, 0);
    while (1)
    {
        get_current(&current_offset_l, &current_offset_r);
        if (-2 <= (current_offset_l - current_l) && (current_offset_l - current_l) <= 2 && -2 <= (current_offset_r - current_r) && (current_offset_r - current_r) <= 2)
        {
            break;
        }
        current_l = current_offset_l;
        current_r = current_offset_r;
    }
    ESP_LOGI("MOTOR CURRENT OFFSET", "L: %d, R: %d", current_offset_l, current_offset_r);
}

void motor_set_freqency(uint32_t frequency)
{
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, frequency));
}

void motor_set_duty(uint32_t duty_l, uint32_t duty_r)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_l));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_r));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

void motor_get_current(int *current_l, int *current_r)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, L_MOTOR_ADC_CHANNEL, current_l));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, R_MOTOR_ADC_CHANNEL, current_r));

    *current_l -= current_offset_l;
    *current_r -= current_offset_r;
}

void motor_init_pid(void)
{
    pid_delta_init(&pid, MOTOR_KP, MOTOR_KI, MOTOR_KD);
}

void motor_set_throttle(int throttle_l, int throttle_r)
{
    int current_l = 0;
    int current_r = 0;
    motor_get_current(&current_l, &current_r);

    throttle_l = throttle_l > MOTOR_THROTTLE_MAX ? MOTOR_THROTTLE_MAX : throttle_l;
    throttle_l = throttle_l < MOTOR_THROTTLE_MIN ? MOTOR_THROTTLE_MIN : throttle_l;
    throttle_r = throttle_r > MOTOR_THROTTLE_MAX ? MOTOR_THROTTLE_MAX : throttle_r;
    throttle_r = throttle_r < MOTOR_THROTTLE_MIN ? MOTOR_THROTTLE_MIN : throttle_r;

    float setpoint_l = (throttle_l - MOTOR_THROTTLE_MIN) / (MOTOR_THROTTLE_MAX - MOTOR_THROTTLE_MIN) * MOTOR_CURRENT_MAX;
    float setpoint_r = (throttle_r - MOTOR_THROTTLE_MIN) / (MOTOR_THROTTLE_MAX - MOTOR_THROTTLE_MIN) * MOTOR_CURRENT_MAX;

    float delta_duty_l = pid_delta_update(&pid, setpoint_l, current_l);
    float delta_duty_r = pid_delta_update(&pid, setpoint_r, current_r);

    duty_l += delta_duty_l;
    duty_r += delta_duty_r;
    duty_l = duty_l > MOTOR_PWM_DUTY_MAX ? MOTOR_PWM_DUTY_MAX : duty_l;
    duty_l = duty_l < MOTOR_PWM_DUTY_MIN ? MOTOR_PWM_DUTY_MIN : duty_l;
    duty_r = duty_r > MOTOR_PWM_DUTY_MAX ? MOTOR_PWM_DUTY_MAX : duty_r;
    duty_r = duty_r < MOTOR_PWM_DUTY_MIN ? MOTOR_PWM_DUTY_MIN : duty_r;

    motor_set_duty(duty_l, duty_r);
}
