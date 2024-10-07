#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define UART_ID uart0

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

// Estrutura para armazenar os dados de aceleração
typedef struct {
    int8_t x;
    int8_t y;
} mouse_data_t;

typedef struct {
    float x;
    float y;
    float z;
} i2c_data;

QueueHandle_t xQueueMouse;

// Função de reset do MPU6050
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

// Função para ler dados brutos do MPU6050
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    // Lê os dados de aceleração
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê os dados do giroscópio
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê a temperatura
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}


// Função para converter os dados do sensor

// Tarefa para leitura e processamento do MPU6050
void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    FusionVector gyroscope_converted;
    FusionVector accelerometer_converted;

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope_conveted;
        gyroscope_conveted.axis.x = gyro[0] / 131.0f;
        gyroscope_conveted.axis.y = gyro[1] / 131.0f;
        gyroscope_conveted.axis.z = gyro[2] / 131.0f;

        FusionVector accelerometer_converted;
        accelerometer_converted.axis.x = acceleration[0] / 16384.0f;
        accelerometer_converted.axis.y = acceleration[1] / 16384.0f;
        accelerometer_converted.axis.z = acceleration[2] / 16384.0f;

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope_converted, accelerometer_converted, 0.01f);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // printf("Roll: %0.1f, Pitch: %0.1f, Yaw: %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        // x
        uart_putc_raw(UART_ID, 0);
        uart_putc_raw(UART_ID, (int) euler.angle.pitch & 0xFF);
        uart_putc_raw(UART_ID, ((int) euler.angle.pitch >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);

        // y

        int y = (euler.angle.roll) * (-1);
        uart_putc_raw(UART_ID, 1);
        uart_putc_raw(UART_ID, y & 0xFF);
        uart_putc_raw(UART_ID, (y >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);



        // printf("Acc. X = %d, Y = %d, Z = %d - ", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d - ", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n\n\n", (temp / 340.0) + 36.53);

        int modulo = abs(acceleration[1]);
        
        if (modulo > 17000){
            // printf("Mouse click detectado!\n");
            uart_putc_raw(UART_ID, 2);
            uart_putc_raw(UART_ID, 0);
            uart_putc_raw(UART_ID, ((modulo >> 8) & 0xFF));
            uart_putc_raw(UART_ID, -1);
        }

        


        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void main() {
    stdio_init_all();

    xQueueMouse = xQueueCreate(10, sizeof(mouse_data_t));

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
