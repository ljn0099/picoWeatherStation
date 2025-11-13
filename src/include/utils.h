#ifndef UTILS_H
#define UTILS_H

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

#define VOLT_TO_ADC(v) ((uint16_t)((v) * ((1 << ADC_BITS) - 1) / ADC_VREF))
#define ADC_TO_VOLT(adc) ((float)(adc) * ADC_VREF / ((1 << ADC_BITS) - 1))

#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)
#define PA_TO_HPA(x) ((x) / 100.0f)

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

#endif
