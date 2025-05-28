#include <Arduino.h>
#include "CvbsAnalyzer.h"

const int k_gpioPin = 36;
const int k_gpioPin1 = 39;
CvbsAnalyzer g_cvbsAnalyzer;

void setup()
{
  Serial.begin(1000000);
  delay(100);
  Serial.printf("Initing CvbsAnalyzer for gpio %d...", k_gpioPin);
  FastADCState fastAdcState = g_cvbsAnalyzer.m_fastAdc.Initialize();
  if (fastAdcState != FastADCState::k_initializedAdcStopped)
  {
    Serial.printf("FastADC::Initialize() failed with state %d!", (int)fastAdcState);
  }
  else
  {
    Serial.printf("FastADC Initialized.");
  }

  g_cvbsAnalyzer.AnalyzePin(k_gpioPin);

}

void loop()
{
}
