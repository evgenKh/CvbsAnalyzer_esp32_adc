#ifndef Histogram_H
#define Histogram_H

#include <array>
#include <algorithm>

template <typename CounterType, typename DataType, size_t k_binsCount>
class Histogram
{
public:
    Histogram(const DataType binsRangeMin, const DataType binsRangeMax) : m_binsRangeMin(binsRangeMin),
                                                                          m_binsRangeMax(binsRangeMax),
                                                                          m_binWidth((binsRangeMax - binsRangeMin) / static_cast<double>(k_binsCount))
    {
        Reset();
    }

    void Reset()
    {
        m_totalCount = 0;
        m_array.fill(0);
    }

    double GetBinLowBound(size_t binIndex) const
    {
        return m_binsRangeMin + binIndex * m_binWidth;
    }

    double GetBinCenter(size_t binIndex) const
    {
        return m_binsRangeMin + (binIndex + 0.5) * m_binWidth;
    }

    double GetBinHighBound(size_t binIndex) const
    {
        return m_binsRangeMin + (binIndex + 1.0) * m_binWidth;
    }

    size_t GetBinIndexForValueWithClamp(DataType sampleValue) const
    {
        const DataType clamped = (sampleValue < m_binsRangeMin ? m_binsRangeMin : (sampleValue > m_binsRangeMax ? m_binsRangeMax : sampleValue));
        
        const int32_t binIndex = static_cast<int32_t>((clamped - m_binsRangeMin) / m_binWidth);
        assert(binIndex >= 0 && binIndex < k_binsCount); // shouldn't happed if we did clamp

        return binIndex;
    }

    int32_t GetBinIndexForValue(DataType sampleValue) const
    {
        const int32_t binIndex = static_cast<int32_t>((sampleValue - m_binsRangeMin) / m_binWidth);
        return binIndex;
    }

    void PushSample(DataType sampleValue)
    {
        const DataType clamped = (sampleValue < m_binsRangeMin ? m_binsRangeMin : (sampleValue > m_binsRangeMax ? m_binsRangeMax : sampleValue));
        const size_t binIndex = GetBinIndexForValue(clamped);

        m_array[binIndex]++;

        // Update MinMax using clamped value
        // So [m_sampleMin, m_sampleMax] is always in range of [m_binsRangeMin, m_binsRangeMax]
        if (clamped < m_sampleMin || (!m_totalCount))
            m_sampleMin = clamped;
        if (clamped > m_sampleMax || (!m_totalCount))
            m_sampleMax = clamped;

        m_totalCount++;
    }

    std::array<CounterType, k_binsCount> m_array;

    const DataType m_binsRangeMin;
    const DataType m_binsRangeMax;
    const double m_binWidth;

    DataType m_sampleMin;
    DataType m_sampleMax;

    CounterType m_totalCount = 0;
};

#endif // Histogram_H