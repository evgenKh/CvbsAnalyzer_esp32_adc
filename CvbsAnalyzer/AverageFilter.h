#ifndef AverageFilter_H
#define AverageFilter_H

#include "Arduino.h"
#include "CvbsAnalyzerGlobals.h"

enum class AverageFilterState : signed char
{
    k_needMoreSamples = 0,
    k_finished,
};

class MeanAverageFilter 
{
public:
    MeanAverageFilter()
    {
        Reset();
    }

    void Reset()
    {
        m_accumulator = 0;
        m_samplesCount = 0;
        m_state = AverageFilterState::k_needMoreSamples;
    }

    // Running average without buffer (exponential moving average)
    AverageFilterState PushSamples(const uint16_t *newData, size_t newDataLen)
    {
        for (size_t i = 0; i < newDataLen; i++)
        {
            m_accumulator += newData[i];
            m_samplesCount++;
            if(m_samplesCount >= k_minSamplesForCalculation) break; 
        }

        if (m_samplesCount < k_minSamplesForCalculation)            
            m_state = AverageFilterState::k_needMoreSamples;
        else            
            m_state = AverageFilterState::k_finished;
        return m_state;
    }

    inline uint16_t GetAverage() const { return m_accumulator / m_samplesCount; }
    inline size_t GetSamplesCount() const { return m_samplesCount; }
    inline AverageFilterState GetState() const { return m_state; }
    inline size_t GetMinSamplesForCalculation() const { return k_minSamplesForCalculation; }

private:
    AverageFilterState m_state;
    size_t m_samplesCount = 0;
    uint32_t m_accumulator = 0;

    constexpr static size_t k_minSamplesForCalculationUs = 100;
    constexpr static size_t k_minSamplesForCalculation = UsToSamplesContexpr(k_minSamplesForCalculationUs);
};


class RunningAverageFilter 
{
public:
    RunningAverageFilter(size_t windowSizeMicroseconds = k_minSamplesForCalculationUs)
    {
        m_windowSizeSamplesFloat = static_cast<double>(UsToSamples(windowSizeMicroseconds));
        Reset();
    }

    void Reset()
    {
        m_average = 0;
        m_samplesCount = 0;
        m_state = AverageFilterState::k_needMoreSamples;
    }

    // Running average without buffer (exponential moving average)
    AverageFilterState PushSamples(const uint16_t *newData, size_t newDataLen)
    {
        for (size_t i = 0; i < newDataLen; i++)
        {
            double sampleVal = static_cast<double>(newData[i]);
            if (m_samplesCount == 0)
            {
                m_average = sampleVal;
            }
            else
            {
                const float alpha = 1.0f / m_windowSizeSamplesFloat;
                m_average = (1.0f - alpha) * m_average + alpha * sampleVal;
            }
            m_samplesCount++;
        }

        if (m_samplesCount < k_minSamplesForCalculation)            
            m_state = AverageFilterState::k_needMoreSamples;
        else            
            m_state = AverageFilterState::k_finished;
        return m_state;
    }

    inline double GetAverage() const { return m_average; }
    inline size_t GetSamplesCount() const { return m_samplesCount; }
    inline AverageFilterState GetState() const { return m_state; }
    inline size_t GetMinSamplesForCalculation() const { return k_minSamplesForCalculation; }

private:
    AverageFilterState m_state;
    size_t m_samplesCount = 0;
    double m_average = 0.0f;

    double m_windowSizeSamplesFloat = 10.0f;

    constexpr static size_t k_minSamplesForCalculationUs = 100;
    constexpr static size_t k_minSamplesForCalculation = UsToSamplesContexpr(k_minSamplesForCalculationUs);
};

#endif // AverageFilter_H