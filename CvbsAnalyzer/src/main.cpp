#include <Arduino.h>
#include "CvbsAnalyzer.h"

const int k_gpioPin = 36;
const int k_gpioPin1 = 39;
CvbsAnalyzer g_cvbsAnalyzer(k_gpioPin);

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

  fastAdcState = g_cvbsAnalyzer.m_fastAdc.StartADCSampling(k_gpioPin);
  if (fastAdcState != FastADCState::k_adcStarted)
  {
    Serial.printf("FastADC::StartADCSampling() for gpioPin %d failed with state %d!", (int)k_gpioPin, (int)fastAdcState);
  }
  else
  {
    Serial.printf("FastADC started for gpioPin %d.", (int)k_gpioPin);
  }

  g_cvbsAnalyzer.m_fastAdc.ReadSamplesBlocking();
  Serial.print("Stopping adc...");

  fastAdcState = g_cvbsAnalyzer.m_fastAdc.StopADCSampling();
  if (fastAdcState != FastADCState::k_initializedAdcStopped)
  {
    Serial.printf("FastADC::StopADCSampling() for gpioPin %d failed with state %d!", (int)k_gpioPin, (int)fastAdcState);
  }
  else
  {
    Serial.printf("FastADC stopped for gpioPin %d.", (int)k_gpioPin);
  }
  delay(10);

}

void loop()
{
}
