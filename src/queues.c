#include "queues.h"
#include "weather_types.h"

queue_t weatherFinalQueue;
queue_t weatherComputeQueue;
queue_t weatherSampleQueue;
queue_t epochTimeQueue;

void queues_init(void) {
    queue_init(&weatherFinalQueue, sizeof(weatherFinal_t), 5);
    queue_init(&weatherComputeQueue, sizeof(computeCmd_t), 16);
    queue_init(&weatherSampleQueue, sizeof(sampleCmd_t), 16);
    queue_init(&epochTimeQueue, sizeof(uint64_t), 1);
}
