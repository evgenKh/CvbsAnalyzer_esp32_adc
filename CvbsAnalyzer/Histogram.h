#ifndef Histogram_H
#define Histogram_H

#include <array>
#include <algorithm>
#include "CvbsAnalyzerGlobals.h"

template <typename CounterType, typename DataType, typename BinWidthType, size_t k_binsCount>
class Histogram : public std::array<CounterType, k_binsCount>
{
public:

    Histogram(const DataType binsRangeMin, const DataType binsRangeMax)
        : m_binsRange(binsRangeMin, binsRangeMax),
          m_binWidth((binsRangeMax - binsRangeMin) / static_cast<BinWidthType>(k_binsCount))
    {
        assert(binsRangeMin < binsRangeMax);
        Reset();
    }

    void Reset()
    {
        m_sampleValuesRange.first = 0;
        m_sampleValuesRange.second = 0;
        m_samplesCount = 0;
        this->fill(0);
    }

    inline BinWidthType GetBinLowBound(size_t binIndex) const
    {
        return m_binsRange.first + binIndex * m_binWidth;
    }

    inline DataType GetBinCenter(size_t binIndex) const
    {
        //if (std::is_floating_point<DataType>::value)
        if(true)
        {
            return static_cast<DataType>(m_binsRange.first + (binIndex * m_binWidth) + (m_binWidth / static_cast<BinWidthType>(2)));
        }
        else
        {
            // for integer types, add 0.5 to round to nearest
            //return static_cast<DataType>(m_binsRange.first + (binIndex + 0.5) * m_binWidth + 0.5);
        }
    }
    inline BinWidthType GetBinCenterPrecise(size_t binIndex) const
    {
        return m_binsRange.first + (binIndex * m_binWidth) + (m_binWidth / static_cast<BinWidthType>(2));
    }

    inline BinWidthType GetBinHighBound(size_t binIndex) const
    {
        return m_binsRange.first + (binIndex + static_cast<BinWidthType>(1)) * m_binWidth;
    }

    inline DataType ClampValue(const DataType sampleValue) const
    {
        return (sampleValue < m_binsRange.first ? m_binsRange.first :
                 (sampleValue > m_binsRange.second ? m_binsRange.second : 
                    sampleValue));
    }


    inline size_t GetBinIndexForValue(DataType sampleValue) const
    {
        //Not clamping value but clamping index!!!
        const int32_t binIndexUnclamped = GetBinIndexForValueNotClampingToBinRange(sampleValue);

        return (binIndexUnclamped < 0 ? 0 :
                 (binIndexUnclamped > k_binsCount-1 ? k_binsCount-1 : 
                    binIndexUnclamped));
    }

    // May return bin index out of range if sampleValue is out of range
    inline int32_t GetBinIndexForValueNotClampingToBinRange(DataType sampleValue) const
    {
        const int32_t binIndex = static_cast<int32_t>((static_cast<BinWidthType>(sampleValue) - static_cast<BinWidthType>(m_binsRange.first)) / m_binWidth);
        return binIndex;
    }

    void PushSample(DataType sampleValue)
    {
        const DataType clamped = ClampValue(sampleValue);
        const size_t binIndex = GetBinIndexForValue(clamped);
        assert(binIndex >= 0 && binIndex < k_binsCount); // shouldn't happed if we did clamp

        this->operator[](binIndex)++;

        // Update MinMax using clamped value
        // So [m_sampleMin, m_sampleMax] is always in range of [m_binsRangeMin, m_binsRangeMax]
        if (clamped < m_sampleValuesRange.first || (!m_samplesCount))
            m_sampleValuesRange.first = clamped;
        if (clamped > m_sampleValuesRange.second || (!m_samplesCount))
            m_sampleValuesRange.second = clamped;

        m_samplesCount++;
    }

    void Extend(const Histogram<CounterType, DataType, BinWidthType, k_binsCount> &other)
    {
        if(!other.GetSamplesCount()) return;
        
        assert(m_binsRange.first == other.m_binsRange.first && m_binsRange.second == other.m_binsRange.second);

        for (size_t i = 0; i < k_binsCount; ++i)
        {
            this->operator[](i) += other[i];
        }
        m_samplesCount += other.m_samplesCount;

        // Update MinMax
        if (other.m_sampleValuesRange.first < m_sampleValuesRange.first || (!m_samplesCount))
            m_sampleValuesRange.first = other.m_sampleValuesRange.first;
        if (other.m_sampleValuesRange.second > m_sampleValuesRange.second || (!m_samplesCount))
            m_sampleValuesRange.second = other.m_sampleValuesRange.second;
    }

    inline CounterType GetSamplesCount() const
    {
        return m_samplesCount;
    }

    static typename std::array<CounterType, k_binsCount>::iterator FindFallingEdge(
        typename std::array<CounterType, k_binsCount>::iterator begin,
        typename std::array<CounterType, k_binsCount>::iterator end,
                                                                CounterType minDelta)
    {
        for(auto it = begin; it != end; ++it)
        {
            if(std::next(it) == end) break;
            if(*std::next(it) < *it &&
                *it - *std::next(it) >= minDelta)
            {
                return it;
            }
        }
        return end; // Not found
    }

    static typename std::array<CounterType, k_binsCount>::iterator FindRisingEdge(
        typename std::array<CounterType, k_binsCount>::iterator begin,
        typename std::array<CounterType, k_binsCount>::iterator end,
                                                                CounterType minDelta)
    {
        for(auto it = begin; it != end; ++it)
        {
            if(std::next(it) == end) break;
            if(*std::next(it) > *it &&
                *std::next(it) - *it >= minDelta)
            {
                return it;
            }
        }
        return end; // Not found
    }

    static typename std::array<CounterType, k_binsCount>::reverse_iterator FindRisingEdge(
        typename std::array<CounterType, k_binsCount>::reverse_iterator begin,
        typename std::array<CounterType, k_binsCount>::reverse_iterator end,
                                                                CounterType minDelta)
    {
        for(auto it = begin; it != end; ++it)
        {
            if(std::next(it) == end) break;
            if(*std::next(it) > *it &&
                *std::next(it) - *it >= minDelta)
            {
                return it;
            }
        }
        return end; // Not found
    }
    
    void Print() const
    {
        CVBS_ANALYZER_LOG("{\n");
        CVBS_ANALYZER_LOG("\t\"m_samplesCount\": %d,\n", m_samplesCount);
        CVBS_ANALYZER_LOG("\t\"k_binsCount\": %d,\n", k_binsCount);
        CVBS_ANALYZER_LOG("\t\"m_binWidth\": %f,\n", m_binWidth);
        CVBS_ANALYZER_LOG("\t\"m_binsRange\": [ %d, %d ],\n", m_binsRange.first, m_binsRange.second);
        CVBS_ANALYZER_LOG("\t\"m_sampleValuesRange\": [ %d, %d ],\n", m_sampleValuesRange.first, m_sampleValuesRange.second);
        CVBS_ANALYZER_LOG("\t\"value_count_pairs\": [\n");
        for(int bin = 0; bin < k_binsCount; bin++)
        {
            if(this->at(bin))
                CVBS_ANALYZER_LOG("\t[ %d, %d ],\n", (int)GetBinCenter(bin), this->at(bin));
        }
        CVBS_ANALYZER_LOG("] },\n");
    }

    //Made this unaccessible since it's not obvoius, no bin or no samples.
    constexpr bool empty() const noexcept = delete;

    const std::pair<const DataType, const DataType> m_binsRange;
    const BinWidthType m_binWidth;

    std::pair<DataType, DataType> m_sampleValuesRange;

    CounterType m_samplesCount = 0;
};



#endif // Histogram_H