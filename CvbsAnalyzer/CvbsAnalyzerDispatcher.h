#ifndef CvbsAnalyzerDispatcher_H
#define CvbsAnalyzerDispatcher_H

#include "CvbsAnalyzerJob.h"
#include "freertos/FreeRTOS.h"

class CvbsAnalyzer;
class CvbsAnalyzerJob;

class CvbsAnalyzerDispatcher
{
public:
    CvbsAnalyzerDispatcher();
    CvbsAnalyzerDispatcher(CvbsAnalyzer* cvbsAnalyzer);
    ~CvbsAnalyzerDispatcher();

    void StartWorkerThread();
    void RequestJob(CvbsAnalyzerJob* jobWithTokens);
private:
    QueueHandle_t m_requestedJobsTokenQueue = nullptr;
    QueueHandle_t m_terminateEventQueue = nullptr;

    TaskHandle_t m_workerThread = nullptr;

    void WorkerThreadLoop();
    CvbsAnalyzer* m_analyzer = nullptr;

};

#endif // CvbsAnalyzerDispatcher_H