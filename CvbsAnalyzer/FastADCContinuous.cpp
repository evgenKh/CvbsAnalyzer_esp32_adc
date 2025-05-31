#include "FastADCContinuous.h"

#if USE_FAST_ADC_CONTINUOUS

FastADC::FastADC()
{

}

FastADCState FastADC::Initialize()
{
    return m_state;
}
FastADCState FastADC::Deinitialize()
{
    return m_state;
}
FastADCState FastADC::StartADCSampling(int8_t gpioPin, bool invertData = false)
{

    return m_state;
}
FastADCState FastADC::StopADCSampling()
{

    return m_state;
}

size_t FastADC::ReadAndPrintSamples()
{

    return 0;
}
size_t FastADC::ReadSamplesBlockingTo(uint16_t* outBuf, size_t bufSizeBytes)
{
    return 0;
}

#endif // USE_FAST_ADC_CONTINUOUS