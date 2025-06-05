#ifndef SamplesPreFilter_H
#define SamplesPreFilter_H

#include "CvbsAnalyzerGlobals.h"
#include "Arduino.h"

class SamplesPreFilter
{
    public:
    SamplesPreFilter(){
        Reset();
    }
    void Reset()
    {
        m_samplesCount = 0;
    }

    size_t PushSamples(const uint16_t *newRawSamples,
         size_t newRawSamplesCount, 
         bool skipLeadingZeroes,
         bool invertData)
    {
        const size_t filteredSamplesCountOld = m_samplesCount;
        const uint16_t xorMask = invertData ? k_adcDataXorMaskForInvert : 0x00;
        bool firstNonZeroSampleFound = false;
        for(size_t i = 0; i < newRawSamplesCount; i += k_adcDataStrideSamples)
        {
            if(m_samplesCount >= k_preFilteredSamplesBufLenSamples)
            {
                break;
            }
            if(skipLeadingZeroes && !firstNonZeroSampleFound)
            {                
                //Due to ADC hacks and calling zeroDma after ADC start, some datasets may have many zero samples at start.    
                if((newRawSamples[i] & k_adcDataMask) == 0)
                {
                    continue; //skip leading zeroes
                }
                firstNonZeroSampleFound = true;
            }

            m_samplesBuf[m_samplesCount] = (newRawSamples[i] ^ xorMask) & k_adcDataMask;
            m_samplesCount++;            
        }
        const size_t samplesAdded = m_samplesCount - filteredSamplesCountOld;
        return samplesAdded;
    }

    void InvertPrefilteredSamples()
    {
        for(size_t i = 0; i < m_samplesCount; i++)
        {
            m_samplesBuf[i] ^= k_adcDataXorMaskForInvert;
        }
    }

    constexpr static size_t k_preFilteredSamplesBufLenSamples = k_dmaBufLenSamples * k_maxDmaReadsPerAnalyzePin/ k_adcDataStrideSamples; 
    
    uint16_t m_samplesBuf[k_preFilteredSamplesBufLenSamples];
    size_t m_samplesCount = 0;
    constexpr static uint16_t k_adcDataMask = 0x0fff; //12 bit width, 0x0fff = 4095, max value for 12 bit ADC
    constexpr static uint16_t k_adcDataXorMaskForInvert = 0x0fff;// apply to data via ^ operato to invert it.(4095-data)
};


#endif // SamplesPreFilter_H

