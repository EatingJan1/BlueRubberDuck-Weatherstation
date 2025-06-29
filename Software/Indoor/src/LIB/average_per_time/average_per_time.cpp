#include "average_per_time.h"

void AveragePerTime::add(float value)
{
    uint32_t now = millis();

    this->cleanup();

    TimedValue entry = { now, value };
    this->values.push_back(entry);
}

float AveragePerTime::getSum(uint32_t periodMs)
{
    float sum = 0.0f;
    uint32_t now = millis();

    for(const auto &entry : this->values)
    {
        if(now - entry.timestamp <= periodMs)
        {
            sum += entry.value;
        }
    }

    return sum;
}

float AveragePerTime::getAverage(uint32_t periodMs)
{
    int count = this->getCount(periodMs);

    if(count == 0)
    {
        return 0.0f;
    }

    return this->getSum(periodMs) / count;
}

int AveragePerTime::getCount(uint32_t periodMs)
{
    int count = 0;
    uint32_t now = millis();

    for(const auto &entry : this->values)
    {
        if(now - entry.timestamp <= periodMs)
        {
            count++;
        }
    }

    return count;
}

void AveragePerTime::cleanup()
{
    uint32_t now = millis();

    this->values.erase(
        std::remove_if(
            this->values.begin(),
            this->values.end(),
            [now, this](const TimedValue &entry)
            {
                return now - entry.timestamp > this->MAX_HISTORY_MS;
            }
        ),
        this->values.end()
    );
}
