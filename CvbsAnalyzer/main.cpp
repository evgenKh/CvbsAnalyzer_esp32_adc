#include <Arduino.h>
#include "CvbsAnalyzer.h"

const int k_gpioPin = 36;
const int k_gpioPin1 = 39;
CvbsAnalyzer g_cvbsAnalyzer;

void setup()
{
  Serial.begin(1000000);
  delay(500);

  CvbsAnalyzerState state = g_cvbsAnalyzer.InitializeFastADC();

  if (state != CvbsAnalyzerState::k_initializedAndIdle)
  {
    CVBS_ANALYZER_LOG("CvbsAnalyzer initialization failed with state %d!\n", (int)state);
    return;
  }
}

void loop()
{
  static int loopNum = 0;
  CVBS_ANALYZER_LOG("Running loop # %d.\n", loopNum);
  CvbsAnalyzerState state = g_cvbsAnalyzer.AnalyzePin(k_gpioPin);
  CVBS_ANALYZER_LOG("CvbsAnalyzer finished with state %d.\n", (int)state);

  delay(1000);
  loopNum++;
}
