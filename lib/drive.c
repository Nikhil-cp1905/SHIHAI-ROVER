#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include "Tarzan/lib/drive.h"

#define PULSE_PER_REV 200 // Define this according to your motor
#define MINUTES_TO_MICRO 60000000

// Define crab modes
#define CRAB_MODE_0_2_1_3 0
#define CRAB_MODE_0_1 1

// Structure for Stepper Motor control
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
void control_motors(uint16_t channel1, uint16_t channel2, uint16_t channel3, uint16_t channel4, uint16_t *channel_range, uint8_t crab_mode) {
    if (crab_mode == CRAB_MODE_0_2_1_3) {
        // Apply crab mode where motors 1 and 3 move together, 2 and 4 together
        Stepper_motor_write(&motor1, channel1, channel_range);
        Stepper_motor_write(&motor3, channel3, channel_range);
        Stepper_motor_write(&motor2, channel2, channel_range);
        Stepper_motor_write(&motor4, channel4, channel_range);
    } else if (crab_mode == CRAB_MODE_0_1) {
        // Apply crab mode where motors 1 and 2 move together, 3 and 4 together
        Stepper_motor_write(&motor1, channel1, channel_range);
        Stepper_motor_write(&motor2, channel2, channel_range);
        Stepper_motor_write(&motor3, channel3, channel_range);
        Stepper_motor_write(&motor4, channel4, channel_range);
    }
}
    

// Main control loop
void main(void) {
    uint16_t channel_values[8] = {0};  // Initialize channel values
    uint16_t channel_range[2] = {1000, 2000}; // Range for motor control
    uint8_t crab_mode = CRAB_MODE_0_1;  // Default crab mode

    // Initialize GPIO pins for motors
    if (init_motor_gpio(&motor1) || init_motor_gpio(&motor2) || init_motor_gpio(&motor3) || init_motor_gpio(&motor4)) {
        printk("Failed to initialize motor GPIOs\n");
        return;
    }

    // Main loop
    while (1) {
        // Retrieve the current SBUS channel values (replace with your actual function to get SBUS data)
        uint16_t *channel_data = parse_buffer(channel_values);

        // Check SBUS channel 7 and 8 for mode switching
        if (channel_data[7] > 1500) {
            crab_mode = CRAB_MODE_0_2_1_3; // Switch to crab mode 0_2_1_3
        } else if (channel_data[8] > 1500) {
            crab_mode = CRAB_MODE_0_1; // Switch to crab mode 0_1
        }

        // Control the motors based on the crab mode and channel values
        control_motors(channel_data[0], channel_data[1], channel_data[2], channel_data[3], channel_range, crab_mode);

        k_msleep(10);  // Adjust this delay based on motor speed requirements
    }
}

