#if USE_CVBS_ANALYZER_MAIN
// Code below if for test application. Don't need it when using as library.

#include <Arduino.h>
#include "CvbsAnalyzer.h"
#include "CvbsAnalyzerDispatcher.h"
#include "CvbsAnalyzerJob.h"

CvbsAnalyzer g_cvbsAnalyzer;
CvbsAnalyzerDispatcher g_cvbsAnalyzerDispatcher(&g_cvbsAnalyzer);
CvbsAnalyzerJob g_pin35Job(35, CvbsAnalyzerJobType::k_videoScore, CvbsAnalyzerInversionType::k_nonInvertedThenInverted);
CvbsAnalyzerJob g_pin36Job(36, CvbsAnalyzerJobType::k_videoScore, CvbsAnalyzerInversionType::k_nonInvertedThenInverted);
CvbsAnalyzerJob g_pin36AverageJob(36, CvbsAnalyzerJobType::k_averageRssi, CvbsAnalyzerInversionType::k_nonInvertedOnly);

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
    //while(!g_pin35Job.IsDone()) {}

    CVBS_ANALYZER_LOG_INFO("m_videoScore.m_isVideo=%f m_videoScoreInverted.m_isVideo=%f\n",
                            g_pin35Job.GetResult().m_videoScore.m_isVideo,
                            g_pin35Job.GetResult().m_videoScoreFromInverted.m_isVideo);

    delay(5);
  }

  if(true)
  {
    CVBS_ANALYZER_LOG_INFO("Reading pin %d\t\t: ", g_pin36Job.m_gpioPin);
    g_cvbsAnalyzerDispatcher.RequestJob(&g_pin36Job);
    g_pin36Job.WaitUntilDone();
    CVBS_ANALYZER_LOG_INFO("m_videoScore.m_isVideo=%f m_videoScoreInverted.m_isVideo=%f\n",
                            g_pin36Job.GetResult().m_videoScore.m_isVideo,
                            g_pin36Job.GetResult().m_videoScoreFromInverted.m_isVideo);
  }

  if(false)
  {
    CVBS_ANALYZER_LOG_INFO("Reading pin avg %d\t\t: ", g_pin36AverageJob.m_gpioPin);
    g_cvbsAnalyzerDispatcher.RequestJob(&g_pin36AverageJob);
    g_pin36AverageJob.WaitUntilDone();
    //while(!g_pin35Job.IsDone()) {}

    CVBS_ANALYZER_LOG_INFO("m_rssiAverage=%d\n",
                            g_pin36AverageJob.GetResult().m_rssiAverage);

    delay(5);
  }

  CVBS_ANALYZER_LOG_INFO("\n");
  //CVBS_ANALYZER_LOG("CvbsAnalyzer finished with state %d.\n", (int)state);

  delay(500);
  loopNum++;
}

#endif // USE_CVBS_ANALYZER_MAIN