#include <Arduino.h>
#include "CvbsAnalyzer.h"

const int k_gpioPin = 36;
CvbsAnalyzer g_cvbsAnalyzer(k_gpioPin);

void setup() {
  Serial.begin(1000000);
  Serial.printf("Initing CvbsAnalyzer for gpio %d...", k_gpioPin);
  //g_cvbsAnalyzer.m_fastAdc.BeginADCI2S();
  g_cvbsAnalyzer.m_fastAdc.BeginADCDetectedVideo();
  Serial.printf("Inited.");
  g_cvbsAnalyzer.m_fastAdc.i2s_read_samples();
}

void loop() {
}
