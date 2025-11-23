#include "queues.h"
#include "weather_types.h"

queue_t weatherFinalQueue;
queue_t weatherComputeQueue;
queue_t weatherSampleQueue;
queue_t epochTimeQueue;
queue_t weatherSerializedQueue;

void queues_init(void) {
    queue_init(&weatherFinalQueue, sizeof(weatherFinal_t), WEATHER_FINAL_QUEUE_SIZE);
    queue_init(&weatherComputeQueue, sizeof(computeCmd_t), WEATHER_COMPUTE_QUEUE_SIZE);
    queue_init(&weatherSampleQueue, sizeof(sampleCmd_t), WEATHER_SAMPLE_QUEUE_SIZE);
    queue_init(&epochTimeQueue, sizeof(uint64_t), 1);
    queue_init(&weatherSerializedQueue, sizeof(payload_t), WEATHER_SERIALIZED_QUEUE_SIZE);
}
