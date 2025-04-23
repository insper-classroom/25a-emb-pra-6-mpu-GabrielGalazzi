#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "Fusion.h"

#define MPU_ADDRESS 0x68
#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define AVG_WINDOW_SIZE 10
#define GESTURE_THRESHOLD 30.0f 

float roll_window[AVG_WINDOW_SIZE] = {0};
float pitch_window[AVG_WINDOW_SIZE] = {0};
int avg_index = 0;
int avg_count = 0;

float compute_average(float *buffer, int count) {
    float sum = 0;
    for (int i = 0; i < count; i++) sum += buffer[i];
    return sum / count;
}

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Calibration
    int16_t accel_bias[3] = {0}, gyro_bias[3] = {0};
    int16_t acceleration[3], gyro[3], temp;
    const int calibration_samples = 500;

    for (int i = 0; i < calibration_samples; i++) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        for (int j = 0; j < 3; j++) {
            accel_bias[j] += acceleration[j];
            gyro_bias[j] += gyro[j];
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int j = 0; j < 3; j++) {
        accel_bias[j] /= calibration_samples;
        gyro_bias[j] /= calibration_samples;
    }

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Remove bias
        for (int j = 0; j < 3; j++) {
            acceleration[j] -= accel_bias[j];
            gyro[j] -= gyro_bias[j];
        }

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
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Rolling average
        roll_window[avg_index] = euler.angle.roll;
        pitch_window[avg_index] = euler.angle.pitch;
        avg_index = (avg_index + 1) % AVG_WINDOW_SIZE;
        if (avg_count < AVG_WINDOW_SIZE) avg_count++;

        euler.angle.roll = compute_average(roll_window, avg_count);
        euler.angle.pitch = compute_average(pitch_window, avg_count);

        // Scale angles (x100 for precision)
        int16_t rollScaled = (int16_t)(euler.angle.roll * 100);
        int16_t pitchScaled = (int16_t)(euler.angle.pitch * 100);

        // Send data (0xFF sync + 4 bytes: roll, pitch)
        uart_putc(UART_ID, 0xFF);
        uart_putc(UART_ID, rollScaled & 0xFF);
        uart_putc(UART_ID, (rollScaled >> 8) & 0xFF);
        uart_putc(UART_ID, pitchScaled & 0xFF);
        uart_putc(UART_ID, (pitchScaled >> 8) & 0xFF);

        // Click gesture (0xFE)
        if (fabsf(euler.angle.pitch) > GESTURE_THRESHOLD) {
            uart_putc(UART_ID, 0xFE);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(mpu6050_task, "MPU6050_Task", 8192, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1);
}