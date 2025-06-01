#include <Arduino.h>
#include "CvbsAnalyzer.h"

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
  CvbsAnalyzerState state;
  
  auto pin35 = 35;
  auto pin36 = 36;
  CVBS_ANALYZER_LOG_INFO("Reading pin %d\t\t: ", pin35);
  state = g_cvbsAnalyzer.AnalyzePin(pin35, false);
  delay(5);

  CVBS_ANALYZER_LOG_INFO("Reading pin %d Inverted: ", pin35);
  state = g_cvbsAnalyzer.AnalyzePin(pin35, true);
  delay(5);

  if(true){
    CVBS_ANALYZER_LOG_INFO("Reading pin %d\t\t: ", pin36);
    state = g_cvbsAnalyzer.AnalyzePin(pin36, false);
    delay(5);

    CVBS_ANALYZER_LOG_INFO("Reading pin %d Inverted: ", pin36);
    state = g_cvbsAnalyzer.AnalyzePin(pin36, true);
    delay(5);
  }

  CVBS_ANALYZER_LOG_INFO("\n");
  //CVBS_ANALYZER_LOG("CvbsAnalyzer finished with state %d.\n", (int)state);

  delay(1000);
  loopNum++;
}
