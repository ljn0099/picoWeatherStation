#ifndef QUEUES_H
#define QUEUES_H

#include "pico/util/queue.h"

#define WEATHER_FINAL_QUEUE_SIZE 5
#define WEATHER_COMPUTE_QUEUE_SIZE 16
#define WEATHER_SAMPLE_QUEUE_SIZE 16

#define COMPUTE_QUEUE_ADD_CMD(cmd) queue_try_add(&weatherComputeQueue, &(computeCmd_t){cmd})

#define SAMPLE_QUEUE_ADD_CMD(cmd) queue_try_add(&weatherSampleQueue, &(sampleCmd_t){cmd})

extern queue_t weatherFinalQueue;
extern queue_t weatherComputeQueue;
extern queue_t weatherSampleQueue;
extern queue_t epochTimeQueue;

void queues_init(void);

#endif
