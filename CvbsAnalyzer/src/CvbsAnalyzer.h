#ifndef CvbsAnalyzer_H
#define CvbsAnalyzer_H

#include "Arduino.h"
#include "FastADC.h"
#include "AmplitudeCaclulator.h"


class CvbsAnalyzer
{
    public:
    CvbsAnalyzer();
    FastADC m_fastAdc;
    AmplitudeCaclulator m_amplitudeCaclulator;
    
    void AnalyzePin(int gpioPin);
};

#endif // CvbsAnalyzer_H