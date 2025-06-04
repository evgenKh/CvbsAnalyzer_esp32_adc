#include "CvbsAnalyzerDispatcher.h"
#include "Arduino.h"
#include "CvbsAnalyzer.h"


CvbsAnalyzerDispatcher::CvbsAnalyzerDispatcher(CvbsAnalyzer* cvbsAnalyzer):
CvbsAnalyzerDispatcher()
{
    m_analyzer = cvbsAnalyzer;
}
CvbsAnalyzerDispatcher::CvbsAnalyzerDispatcher()
{
    m_requestedJobsTokenQueue = xQueueCreate(100, sizeof(JobQueueToken));
    m_terminateEventQueue = xQueueCreate(1, sizeof(bool));
}

CvbsAnalyzerDispatcher::~CvbsAnalyzerDispatcher()
{
    vQueueDelete(m_requestedJobsTokenQueue);
    m_requestedJobsTokenQueue = nullptr;
    vQueueDelete(m_terminateEventQueue);
    m_terminateEventQueue = nullptr;

    if (m_workerThread)
    {
        vTaskDelete(m_workerThread);
        m_workerThread = nullptr;
    }
}

void CvbsAnalyzerDispatcher::StartWorkerThread()
{
    if(m_workerThread) return;

    int err = xTaskCreatePinnedToCore(
        [](void* arg) {
            static_cast<CvbsAnalyzerDispatcher*>(arg)->WorkerThreadLoop();
        },
        "CvbsAnalyzerTh",
        2000, //stackDepth
        this,
        1, //priority
        &m_workerThread,
        (1 - ARDUINO_RUNNING_CORE) // run on other core.
    );

    if (err != pdPASS) {
        CVBS_ANALYZER_LOG_INFO("Failed to create worker thread: %d\n", err);
        m_workerThread = nullptr;
    }
}

void CvbsAnalyzerDispatcher::RequestJob(CvbsAnalyzerJob *jobWithTokens)
{
    assert(jobWithTokens);
    if(!jobWithTokens->IsDone()){
        return;
    }

    JobQueueToken tokenFromJob = jobWithTokens->TakeToken();
    int err = xQueueSend(m_requestedJobsTokenQueue, &tokenFromJob, portMAX_DELAY);
    assert(err == pdTRUE); //Queue full, this should not happen
}

void CvbsAnalyzerDispatcher::WorkerThreadLoop()
{
    bool terminateEvent = false;
    while(xQueueReceive(m_terminateEventQueue, &terminateEvent, 0) != pdTRUE) // Loop until termination event is received
    {
        JobQueueToken jobToExecuteToken;
        constexpr TickType_t k_popTokenTimeout =  pdMS_TO_TICKS(1);//Lets' release CPU if no jobs for 10ms
        
        if (xQueueReceive(m_requestedJobsTokenQueue, &jobToExecuteToken, k_popTokenTimeout) == pdTRUE)
        {
            assert(jobToExecuteToken.m_job);
            CvbsAnalyzerJob* jobToExecute = jobToExecuteToken.m_job;
            
            m_analyzer->AnalyzePin(jobToExecute->m_gpioPin);
            jobToExecute->m_videoScore = m_analyzer->GetVideoScore();
            jobToExecute->m_videoScoreInverted = m_analyzer->GetVideoScore();

            jobToExecute->ReturnToken(jobToExecuteToken); // Return the token back to the source queue
        }
    }











}
