#include "CvbsAnalyzer.h"

CvbsAnalyzer::CvbsAnalyzer()
{
}

void CvbsAnalyzer::AnalyzePin(int gpioPin)
{
    FastADCState fastAdcState = m_fastAdc.StartADCSampling(gpioPin);
    if (fastAdcState != FastADCState::k_adcStarted)
    {
        Serial.printf("FastADC::StartADCSampling() for gpioPin %d failed with state %d!", (int)gpioPin, (int)fastAdcState);
    }
    else
    {
        Serial.printf("FastADC started for gpioPin %d.", (int)gpioPin);
    }

    {        
        static int16_t buf[400];
        
        for(int run=0;run<5;run++)
        {
            size_t bytesRead = m_fastAdc.ReadSamplesBlockingTo(buf, sizeof(buf));
            if(bytesRead > 100)
            {
                m_amplitudeCaclulator.PushSamples(buf, bytesRead/sizeof(int16_t));

                if(m_amplitudeCaclulator.m_state == AmplitudeCaclulatorState::k_finished)
                {
                    m_syncIntervalsCalculator.PushSamples(buf, bytesRead/sizeof(int16_t), m_amplitudeCaclulator.m_syncTreshold);
                }

                for (int i = 0; i < bytesRead / 2; i++) {
                    printf("%d\n", buf[i] & 0x0fff);
                }
                printf("%d samples printed.(run %d) ----------------\n", bytesRead / 2, run);
            }
        }
    }

    Serial.print("Stopping adc...");

    fastAdcState = m_fastAdc.StopADCSampling();
    if (fastAdcState != FastADCState::k_initializedAdcStopped)
    {
        Serial.printf("FastADC::StopADCSampling() for gpioPin %d failed with state %d!", (int)gpioPin, (int)fastAdcState);
    }
    else
    {
        Serial.printf("FastADC stopped for gpioPin %d.", (int)gpioPin);
    }
    delay(10);
}
