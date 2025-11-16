#ifndef WEATHER_PROTOCOL_H
#define WEATHER_PROTOCOL_H

#include "include/weather_types.h"
#include <stdint.h>

// Caller must free data
uint8_t *create_weather_payload(weatherFinal_t *data, size_t *outLen);

#endif
