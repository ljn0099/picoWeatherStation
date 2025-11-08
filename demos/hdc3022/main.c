
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "sensor/hdc3022.h"

#define I2C_BUS i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_ADDRESS HDC3022_DEFAULT_ADDR
#define I2C_SPEED 100000

#define MEAS_TIME_MS 10000 // 10 sec
#define SAMPLE_TIME_MS 3000 // 3 sec
#define MEAS_TIME_S (MEAS_TIME_MS / 1000)
#define SAMPLE_TIME_S (SAMPLE_TIME_MS / 1000)

typedef struct {
    float sum;
    uint32_t count;
    float average;
} tempData_t;

typedef struct {
    float sum;
    uint32_t count;
    float average;
} humidityData_t;

hdc3022_t hdc3022;
tempData_t tempData = {0};
humidityData_t humidityData = {0};

volatile bool readSample = false;
volatile bool readMeas = false;

bool sample_callback(__unused struct repeating_timer *t) {
    readSample = true;
}

bool meas_callback(__unused struct repeating_timer *t) {
    readMeas = true;
}

int main() {
    stdio_init_all();

    i2c_init(I2C_BUS, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    if (!hdc3022_init_struct(&hdc3022, I2C_BUS, I2C_ADDRESS, HDC3022_AUTO_MEAS_NONE,
                             true)) { // Auto detect default mode
        printf("Error to init the struct\n");
        return -1;
    }
    if (!hdc3022_soft_reset(&hdc3022))
        return -1;
    if (!hdc3022_set_auto_meas_mode(&hdc3022, HDC3022_AUTO_MEAS_1X_1S_LPM_0 ))
        return -1;

    struct repeating_timer measTimer;
    add_repeating_timer_ms(-MEAS_TIME_MS, meas_callback, NULL, &measTimer);
    struct repeating_timer sampleTimer;
    add_repeating_timer_ms(-SAMPLE_TIME_MS, sample_callback, NULL, &sampleTimer);

    while (1) {
        if (readSample) {
            readSample = false;

            float temp, humidity;
            
            if (!hdc3022_readout_auto_meas(&hdc3022, HDC3022_AUTO_MEAS_READOUT, HDC3022_TEMP_CELSIUS,
                                               &temp, &humidity)) { // Do a readout
                    printf("Error doing the readout\n");
                    return -1;
            }
            tempData.sum += temp;
            tempData.count++;
            printf("Temperature: %f\n", temp);

            humidityData.sum += humidity;
            humidityData.count++;
            printf("Humidity: %f\n", humidity);
        }
        if (readMeas) {
            readMeas = false;

            tempData.average = tempData.sum / tempData.count;
            humidityData.average = humidityData.sum / humidityData.count;

            printf("Temp average: %f\n", tempData.average);
            printf("Humidity average: %f\n", humidityData.average);

            tempData.sum = 0;
            tempData.count = 0;
            humidityData.sum = 0;
            humidityData.count = 0;
        }
    }
    return 0;
}
