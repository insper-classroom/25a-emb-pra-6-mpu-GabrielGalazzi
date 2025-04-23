#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period
#define GESTURE_THRESHOLD 15000 // Lembrar de ajustar

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define AVG_WINDOW_SIZE 10

float roll_window[AVG_WINDOW_SIZE] = {0};
float pitch_window[AVG_WINDOW_SIZE] = {0};
int avg_index = 0;
int avg_count = 0;

float compute_average(float *buffer, int count) {
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    return sum / count;
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // I2C configuration
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    // Fusion AHRS setup
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    // Timing

    // Data arrays
    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler raw_euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Fill rolling buffer
        roll_window[avg_index] = raw_euler.angle.roll;
        pitch_window[avg_index] = raw_euler.angle.pitch;
        
        avg_index = (avg_index + 1) % AVG_WINDOW_SIZE;
        if (avg_count < AVG_WINDOW_SIZE) avg_count++;
        
        // Compute averaged Euler angles
        FusionEuler euler;
        euler.angle.roll = compute_average(roll_window, avg_count);
        euler.angle.pitch = compute_average(pitch_window, avg_count);
        // euler.angle.yaw = raw_euler.angle.yaw; 
        float pitchDelta = raw_euler.angle.pitch;

        // Scaled roll and pitch
        int16_t rollScaled = (int16_t)(euler.angle.roll * 100);
        int16_t pitchScaled = (int16_t)(euler.angle.pitch * 100);
        // int16_t yawScaled = (int16_t)(euler.angle.yaw * 100);


        // Sync byte
        putchar_raw(0xFF);

        // Send roll
        putchar_raw(rollScaled & 0xFF);
        putchar_raw((rollScaled >> 8) & 0xFF);
        

        // Send pitch
        putchar_raw(pitchScaled & 0xFF);
        putchar_raw((pitchScaled >> 8) & 0xFF);

        



        // Gesture marker (optional, only when triggered)
        if (pitchDelta > 100.0f) {
            putchar_raw(0xFE);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
