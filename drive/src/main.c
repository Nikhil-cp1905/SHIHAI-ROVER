#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <Tarzan/lib/kyvernitis.h> 
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Motor Definitions */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
    {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                  \
     .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                              \
     .max_pulse = DT_PROP(pwm_dev_id, max_pulse)}

struct pwm_motor motor[8] = {
    DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)
};

/* Stepper Motor Configuration */
const struct stepper_motor stepper[4] = {
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor1), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor2), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor3), step_gpios)},
    {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), dir_gpios),
     .step = GPIO_DT_SPEC_GET(DT_ALIAS(stepper_motor4), step_gpios)}
};

/* UART Configuration */
K_MSGQ_DEFINE(uart_msgq, 25 * sizeof(uint8_t), 10, 1);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // SBUS data
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); // Debugger

/* Control Ranges */
static float linear_velocity_range[] = {-1.5, 1.5};
static float angular_velocity_range[] = {-5.5, 5.5};
static float wheel_velocity_range[] = {-10.0, 10.0};
static uint32_t pwm_range[] = {1120000, 1880000};
static float angle_range[] = {-270.0, 270.0};
static uint16_t channel_range[] = {172, 1811};

uint16_t *ch;       // SBUS channel data
uint8_t packet[25]; // SBUS packet buffer
int pos[2] = {0};   // Stepper motor positions

/* Crab Modes */
enum CrabMode { CRAB_MODE_0_2_1_3, CRAB_MODE_0_1 };
enum CrabMode crab_mode = CRAB_MODE_0_2_1_3;

/* Function to Write Stepper Motor State */
static int stepper_motor_write(const struct stepper_motor *motor, uint16_t ch, int pos) {
    if (abs(ch - 992) < 200)
        return pos;

    gpio_pin_set_dt(&(motor->dir), (ch > 1004) ? 1 : 0);
    pos += (ch > 1004) ? 1 : -1;

    gpio_pin_set_dt(&(motor->step), ((pos & 0x03) == 1 || (pos & 0x03) == 2) ? 1 : 0);
    return pos;
}

/* Function for Crab Motion Control */
static int crab_motion(float direction) {
    int err = 0;
    float interpolated_pwm;

    switch (crab_mode) {
    case CRAB_MODE_0_2_1_3:
        interpolated_pwm = velocity_pwm_interpolation(direction, wheel_velocity_range, pwm_range);
        err |= pwm_motor_write(&(motor[4]), interpolated_pwm);
        err |= pwm_motor_write(&(motor[6]), interpolated_pwm);
        err |= pwm_motor_write(&(motor[5]), velocity_pwm_interpolation(-direction, wheel_velocity_range, pwm_range));
        err |= pwm_motor_write(&(motor[7]), velocity_pwm_interpolation(-direction, wheel_velocity_range, pwm_range));
        break;

    case CRAB_MODE_0_1:
        interpolated_pwm = velocity_pwm_interpolation(direction, wheel_velocity_range, pwm_range);
        err |= pwm_motor_write(&(motor[4]), interpolated_pwm);
        err |= pwm_motor_write(&(motor[5]), interpolated_pwm);
        break;

    default:
        LOG_ERR("Unknown crab mode selected");
        return -1;
    }

    return err;
}

/* Arm Joint Handling Logic */
void arm_joints(struct k_work *work) {
    uint16_t cmd[2] = {ch[4], ch[5]};
    for (int i = 0; i < 2; i++) {
        pos[i] = stepper_motor_write(&stepper[i], cmd[i], pos[i]);
    }
}
K_WORK_DEFINE(my_work, arm_joints);

void my_timer_handler(struct k_timer *dummy) {
    k_work_submit(&my_work);
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

/* UART Data Processing Callback */
void serial_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    uint8_t c, start = 0x0F;

    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev))
        return;

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (c != start) continue;
        packet[0] = c;
        if (uart_fifo_read(uart_dev, packet + 1, 24) == 24) {
            k_msgq_put(&uart_msgq, packet, K_NO_WAIT);
        }
    }
}

/* Main Application Entry */
int main() {
    int err;
    uint16_t neutral = 992;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }

    for (size_t i = 0; i < ARRAY_SIZE(motor); i++) {
        if (!pwm_is_ready_dt(&(motor[i].dev_spec))) {
            LOG_ERR("PWM motor %s is not ready", motor[i].dev_spec.dev->name);
            return -1;
        }
        pwm_motor_write(&(motor[i]), 1500000);
    }

    err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    if (err < 0) {
        LOG_ERR("Error setting UART callback: %d", err);
        return err;
    }
    uart_irq_rx_enable(uart_dev);

    while (1) {
        k_msleep(100);
        if (k_msgq_get(&uart_msgq, &packet, K_NO_WAIT) == 0) {
            ch = parse_sbus_packet(packet);

            float crab_direction = (float)(ch[6] - 992) / 819;
            crab_mode = (ch[7] > 1200) ? CRAB_MODE_0_1 : CRAB_MODE_0_2_1_3;

            err = crab_motion(crab_direction);
            if (err) {
                LOG_ERR("Error executing crab motion: %d", err);
            }

            if (abs(ch[4] - neutral) > 200 || abs(ch[5] - neutral) > 200) {
                k_timer_start(&my_timer, K_MSEC(10), K_NO_WAIT);
            }
        }
    }
    return 0;
}

