#ifndef WEATHER_TYPES_H
#define WEATHER_TYPES_H

#include "sensor/dps310.h"
#include "sensor/hdc3022.h"
#include "sensor/ltr390.h"
#include "sensor/pcf8523.h"
#include <stdbool.h>
#include <time.h>

typedef struct {
    float value;
    bool valid;
} finalData_t;

typedef struct {
    float sum;
    uint32_t count;
} avgData_t;

typedef struct {
    float sumXRad;
    float sumYRad;
    float lastDirRad;
    uint32_t count;
} windDirection_t;

typedef struct {
    volatile uint32_t interrupts;
} rainfall_t;

typedef struct {
    volatile uint32_t interrupts;
} windSpeed_t;

typedef struct {
    volatile uint32_t interrupts;
    uint32_t peakInterrupts;
    float peakDirectionRad;
} peakWind_t;

// Global struct because its for IRQs
typedef struct {
    rainfall_t rainfall;
    windSpeed_t windSpeed;
    peakWind_t peakWind;
} weatherIrq_t;

// Struct for sensor internals
typedef struct {
    hdc3022_t hdc3022;
    dps310_t dps310;
    ltr390_t ltr390;
    pcf8523_t pcf8523;
} weatherSensor_t;

// Struct for averages accumulation
typedef struct {
    windDirection_t windDirection;
    avgData_t temperature;
    avgData_t humidity;
    avgData_t pressure;
    avgData_t lux;
    avgData_t uvi;
    avgData_t solarIrradiance;
} weatherAverage_t;

// Struct with the final data
typedef struct {
    uint64_t epochTimeStart;
    uint64_t epochTimeEnd;

    finalData_t windSpeedKmH;
    finalData_t windDirectionDeg;

    finalData_t peakWindSpeedKmH;
    finalData_t peakWindDirectionDeg;

    finalData_t rainfallMM;

    finalData_t temperatureCelsius;
    finalData_t humidityRh;
    finalData_t uvi;
    finalData_t lux;
    finalData_t pressureHPA;
    finalData_t solarIrradianceWm2;
} weatherFinal_t;

// Wind speed and rainfall don't need to be sampled in the callback because they
// are sampled in the IRQ
typedef enum {
    SAMPLE_HDC3022,
    SAMPLE_DPS310,
    SAMPLE_LTR390,
    SAMPLE_PEAK_WIND,
    SAMPLE_WIND_DIRECTION,
    SAMPLE_PEAK_WIND_DIRECTION
} sampleCmd_t;

typedef enum {
    COMPUTE_TEMP,
    COMPUTE_HUMIDITY,
    COMPUTE_PRES,
    COMPUTE_LUX,
    COMPUTE_UVI,
    COMPUTE_WIND_SPEED,
    COMPUTE_PEAK_WIND,
    COMPUTE_WIND_DIRECTION,
    COMPUTE_RAINFALL,
    COMPUTE_SOLAR_IRRADIANCE,
    COMPUTE_TIMESTAMP,
    COMPUTE_SEND_DATA,
    COMPUTE_TIME_SEND_REQ
} computeCmd_t;

typedef struct {
    uint8_t msg[128];
    size_t len;
} payload_t;

typedef enum { NTP_SUCCESS, NTP_FAILURE } time_alert_t;

typedef enum { NTP_REQUEST } time_request_t;

typedef struct {
    time_alert_t alert;
    struct timespec offset;
} time_msg_t;

#endif
