#include "include/weather_types.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "weather.pb.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ASSIGN_FLOAT_FIELD(MSG, FIELD, DATA)                                                       \
    do {                                                                                           \
        if ((DATA).valid) {                                                                        \
            (MSG).FIELD.value = (DATA).value;                                                      \
            (MSG).has_##FIELD = true;                                                              \
        }                                                                                          \
        else {                                                                                     \
            (MSG).has_##FIELD = false;                                                             \
        }                                                                                          \
    } while (0)

bool create_weather_payload(weatherFinal_t *data, uint8_t *outBuf, size_t outBufSize,
                            size_t *outLen) {
    if (!data || !outBuf || !outLen)
        return false;

    weather_WeatherMeasurement meas = weather_WeatherMeasurement_init_zero;
    meas.periodStart = data->epochTimeStart;
    meas.periodEnd = data->epochTimeEnd;

    ASSIGN_FLOAT_FIELD(meas, temperature, data->temperatureCelsius);
    ASSIGN_FLOAT_FIELD(meas, humidity, data->humidityRh);
    ASSIGN_FLOAT_FIELD(meas, pressure, data->pressureHPA);
    ASSIGN_FLOAT_FIELD(meas, lux, data->lux);
    ASSIGN_FLOAT_FIELD(meas, uvi, data->uvi);
    ASSIGN_FLOAT_FIELD(meas, windSpeed, data->windSpeedKmH);
    ASSIGN_FLOAT_FIELD(meas, windDirection, data->windDirectionDeg);
    ASSIGN_FLOAT_FIELD(meas, gustSpeed, data->peakWindSpeedKmH);
    ASSIGN_FLOAT_FIELD(meas, gustDirection, data->peakWindDirectionDeg);
    ASSIGN_FLOAT_FIELD(meas, rainfall, data->rainfallMM);
    ASSIGN_FLOAT_FIELD(meas, solarIrradiance, data->solarIrradianceWm2);

    size_t estimatedSize;

    if (!pb_get_encoded_size(&estimatedSize, weather_WeatherMeasurement_fields, &meas)) {
        return false;
    }

    if (estimatedSize > outBufSize)
        return false;

    pb_ostream_t stream = pb_ostream_from_buffer(outBuf, outBufSize);

    if (!pb_encode(&stream, weather_WeatherMeasurement_fields, &meas)) {
        return false;
    }

    *outLen = stream.bytes_written;

    return true;
}
