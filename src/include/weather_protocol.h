#ifndef WEATHER_PROTOCOL_H
#define WEATHER_PROTOCOL_H

#include "include/weather_types.h"
#include <stdbool.h>
#include <stdint.h>

bool create_weather_payload(weatherFinal_t *data, uint8_t *outBuf, size_t outBufSize,
                            size_t *outLen);

#endif
