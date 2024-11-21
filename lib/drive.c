#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include "Tarzan/lib/drive.h"

#define PULSE_PER_REV 200 // Define this according to your motor
#define MINUTES_TO_MICRO 60000000

struct StepperMotor {
    struct gpio_dt_spec dir;
    struct gpio_dt_spec step;
    int position; // Track the step position for each motor
};

// Initialize motors
struct StepperMotor motor1 = {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(motor1_dir), gpios),
                              .step = GPIO_DT_SPEC_GET(DT_ALIAS(motor1_step), gpios),
                              .position = 0},
                    motor2 = {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(motor2_dir), gpios),
                              .step = GPIO_DT_SPEC_GET(DT_ALIAS(motor2_step), gpios),
                              .position = 0},
                    motor3 = {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(motor3_dir), gpios),
                              .step = GPIO_DT_SPEC_GET(DT_ALIAS(motor3_step), gpios),
                              .position = 0},
                    motor4 = {.dir = GPIO_DT_SPEC_GET(DT_ALIAS(motor4_dir), gpios),
                              .step = GPIO_DT_SPEC_GET(DT_ALIAS(motor4_step), gpios),
                              .position = 0};

// Helper function to initialize GPIOs
int init_motor_gpio(struct StepperMotor *motor) {
    int ret = gpio_pin_configure_dt(&motor->dir, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&motor->step, GPIO_OUTPUT_ACTIVE);
    return ret;
}

// Interpolate SBUS channel to velocity (keep function as-is)

// Interpolate SBUS channel to PWM (keep function as-is)

// Wrapper to control stepper motor movement based on SBUS channel value
int Stepper_motor_write(struct StepperMotor *motor, uint16_t channel, uint16_t *channel_range) {
    int ret = 0;
    if(channel > channel_range[1]) {
        gpio_pin_set_dt(&(motor->dir), 1); // Set direction to forward
        motor->position += 1; // Move forward
    } else {
        gpio_pin_set_dt(&(motor->dir), 0); // Set direction to backward
        motor->position -= 1; // Move backward
    }

    // Step sequence
    switch(motor->position & 0x03) { // Cycle through steps
        case 0: ret += gpio_pin_set_dt(&(motor->step), 0); break;
        case 1: ret += gpio_pin_set_dt(&(motor->step), 1); break;
        case 2: ret += gpio_pin_set_dt(&(motor->step), 1); break;
        case 3: ret += gpio_pin_set_dt(&(motor->step), 0); break;
    }
    return ret;
}

// Control all motors
void control_motors(uint16_t channel1, uint16_t channel2, uint16_t channel3, uint16_t channel4, uint16_t *channel_range) {
    Stepper_motor_write(&motor1, channel1, channel_range);
    Stepper_motor_write(&motor2, channel2, channel_range);
    Stepper_motor_write(&motor3, channel3, channel_range);
    Stepper_motor_write(&motor4, channel4, channel_range);
}

// Main control loop
void main(void) {
    uint16_t channel_values[4] = { /* Initialize with your channel values */ };
    uint16_t channel_range[2] = { 1000, 2000 };

    // Initialize GPIO pins
    if (init_motor_gpio(&motor1) || init_motor_gpio(&motor2) || init_motor_gpio(&motor3) || init_motor_gpio(&motor4)) {
        printk("Failed to initialize motor GPIOs\n");
        return;
    }

    while (1) {
        control_motors(channel_values[0], channel_values[1], channel_values[2], channel_values[3], channel_range);
        k_msleep(10); // Adjust this delay based on motor speed requirements
    }
}

/*Define PULSE_PER_REV and MINUTES_TO_MICRO constants; initialize StepperMotor struct with dir, step, position fields.

Create motor1, motor2, motor3, motor4 instances with direction and step GPIO pins from device tree.

Define init_motor_gpio to configure dir and step GPIOs for each motor as output.

Set motor direction based on channel range; increment/decrement position; toggle step pin in 4-phase sequence for stepping.

Call Stepper_motor_write for motor1, motor2, motor3, motor4 with respective channel values and range.

Initialize motor GPIOs; if initialization fails, print error; loop continuously to read channels, control motors, and delay 10 ms.
/*
