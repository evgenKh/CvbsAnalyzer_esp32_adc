#include <Arduino.h>
#include "CvbsAnalyzer.h"
#include "CvbsAnalyzerDispatcher.h"
#include "CvbsAnalyzerJob.h"

CvbsAnalyzer g_cvbsAnalyzer;
CvbsAnalyzerDispatcher g_cvbsAnalyzerDispatcher(&g_cvbsAnalyzer);
CvbsAnalyzerJob g_pin35Job(CvbsAnalyzerJobType::k_videoScore, 35);
CvbsAnalyzerJob g_pin36Job(CvbsAnalyzerJobType::k_videoScore, 36);

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
  g_cvbsAnalyzerDispatcher.StartWorkerThread();
}

void loop()
{
  static int loopNum = 0;
  CVBS_ANALYZER_LOG("Running loop # %d.\n", loopNum);
  CvbsAnalyzerState state;
  
  if(true)
  {
    CVBS_ANALYZER_LOG_INFO("Reading pin %d\t\t: ", g_pin35Job.m_gpioPin);
    g_cvbsAnalyzerDispatcher.RequestJob(&g_pin35Job);
    g_pin35Job.WaitUntilDone();
    CVBS_ANALYZER_LOG_INFO("m_videoScore.m_isVideo=%f m_videoScoreInverted.m_isVideo=%f\n",
                            g_pin35Job.m_videoScore.m_isVideo,
                            g_pin35Job.m_videoScoreInverted.m_isVideo);

    delay(5);
  }

  if(true)
  {
    CVBS_ANALYZER_LOG_INFO("Reading pin %d\t\t: ", g_pin36Job.m_gpioPin);
    g_cvbsAnalyzerDispatcher.RequestJob(&g_pin36Job);
    g_pin36Job.WaitUntilDone();
    CVBS_ANALYZER_LOG_INFO("m_videoScore.m_isVideo=%f m_videoScoreInverted.m_isVideo=%f\n",
                            g_pin36Job.m_videoScore.m_isVideo,
                            g_pin36Job.m_videoScoreInverted.m_isVideo);
  }
  
  CVBS_ANALYZER_LOG_INFO("\n");
  //CVBS_ANALYZER_LOG("CvbsAnalyzer finished with state %d.\n", (int)state);

  delay(1000);
  loopNum++;
}
