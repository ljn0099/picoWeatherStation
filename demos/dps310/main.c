#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"
#include "sensor/dps310.h"

#define I2C_BUS i2c1
#define I2C_SDA 18
#define I2C_SCL 19
#define I2C_ADDRESS 0x77
#define I2C_SPEED 100000

#define MEAS_TIME_MS 10000  // 10 sec
#define SAMPLE_TIME_MS 3000 // 3 sec
#define MEAS_TIME_S (MEAS_TIME_MS / 1000)
#define SAMPLE_TIME_S (SAMPLE_TIME_MS / 1000)

typedef struct {
    float sum;
    uint32_t count;
    float average;
} presData_t;

dps310_t dps310;
presData_t presData = {0};

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

    if (!dps310_init_struct(&dps310, I2C_ADDRESS, I2C_BUS)) {
        printf("Error initializing the struct\n");
        return -1;
    }
    if (!dps310_wait_until_ready(&dps310, false, true))
        return -1;
    if (!dps310_soft_reset(&dps310))
        return -1;
    if (!dps310_wait_until_ready(&dps310, true, true))
        return -1;

    if (!dps310_read_temp_calibration(&dps310))
        return -1;
    if (!dps310_read_pres_calibration(&dps310))
        return -1;
    if (!dps310_auto_set_temp_source(&dps310))
        return -1;
    if (!dps310_correct_temp(&dps310))
        return -1;

    if (!dps310_set_pres_oversampling(&dps310, DPS310_OVERSAMPLING_16_TIMES))
        return -1;
    if (!dps310_set_temp_oversampling(&dps310, DPS310_OVERSAMPLING_1_TIME))
        return -1;

    if (!dps310_set_meas_mode(&dps310, DPS310_MODE_CONT_PRES_TEMP_MEAS))
        return -1;

    struct repeating_timer measTimer;
    add_repeating_timer_ms(-MEAS_TIME_MS, meas_callback, NULL, &measTimer);
    struct repeating_timer sampleTimer;
    add_repeating_timer_ms(-SAMPLE_TIME_MS, sample_callback, NULL, &sampleTimer);

    while (1) {
        if (readSample) {
            readSample = false;

            float pres;

            if (!dps310_read_pres(&dps310, &pres, true)) {
                printf("Error doing the readout\n");
                return -1;
            }
            presData.sum += pres;
            presData.count++;
            printf("Pressure: %f\n", pres);
        }
        if (readMeas) {
            readMeas = false;

            presData.average = presData.sum / presData.count;

            printf("Pressure average: %f\n", presData.average);

            presData.sum = 0;
            presData.count = 0;
        }
    }
    return 0;
}
