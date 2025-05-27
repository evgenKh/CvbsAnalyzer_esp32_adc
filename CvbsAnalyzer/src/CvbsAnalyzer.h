#ifndef CvbsAnalyzer_H
#define CvbsAnalyzer_H

#include "Arduino.h"
#include "FastADC.h"


class CvbsAnalyzer
{
    public:
    CvbsAnalyzer(int gpioPin);
    FastADC m_fastAdc;


    private:
    int m_gpioPin;
};

#endif // CvbsAnalyzer_H