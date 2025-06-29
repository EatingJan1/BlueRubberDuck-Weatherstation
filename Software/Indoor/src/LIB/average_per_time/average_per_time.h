#ifndef AVERAGE_PER_TIME_H
#define AVERAGE_PER_TIME_H

#include <Arduino.h>
#include <vector>
#include <algorithm>

typedef struct
{
    uint32_t timestamp;
    float value;
} TimedValue;

class AveragePerTime
{
public:
    AveragePerTime() {};

    void add(float value);
    float getSum(uint32_t periodMs);
    float getAverage(uint32_t periodMs);
    int getCount(uint32_t periodMs);

private:
    std::vector<TimedValue> values;

    const uint32_t MAX_HISTORY_MS = 5 * 60 * 60 * 1000UL; // 5 Stunden

    void cleanup();
};

#endif
