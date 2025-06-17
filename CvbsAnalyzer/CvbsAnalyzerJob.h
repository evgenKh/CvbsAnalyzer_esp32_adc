#ifndef CvbsAnalyzerJob_H
#define CvbsAnalyzerJob_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "VideoScore.h"

enum class CvbsAnalyzerJobType
{
    k_none = 0,
    k_videoScore,
    k_averageRssi
};


enum class CvbsAnalyzerInversionType
{
    k_nonInvertedOnly = 0,
    k_invertedOnly,
    k_nonInvertedThenInverted,
};

class CvbsAnalyzerJob;
struct JobQueueToken
{
    CvbsAnalyzerJob* m_job = nullptr;
};

class CvbsAnalyzerJob
{
public:    

    struct Result
    {        
        VideoScore m_videoScore;
        VideoScore m_videoScoreFromInverted;
        uint16_t m_rssiAverage = 0;
        CvbsAnalyzerState m_cvbsAnalyzerState; 
    };

    CvbsAnalyzerJob(int gpioPin, CvbsAnalyzerJobType type, CvbsAnalyzerInversionType inversionType):
        CvbsAnalyzerJob()
    {
        m_gpioPin = gpioPin;
        m_type = type;
        m_inversionType = inversionType;
    }

    CvbsAnalyzerJob(const CvbsAnalyzerJob& other) = delete; //non copyable!
    CvbsAnalyzerJob& operator=(const CvbsAnalyzerJob& other) = delete; //non copyable!

    CvbsAnalyzerJob()
    {
        m_tokensSourceQueue = xQueueCreate(k_tokensPerJob, sizeof(JobQueueToken));
        assert(m_tokensSourceQueue);

        // Produce tokens that will be moved back and forth and fill source queue with it.
        JobQueueToken myToken;
        myToken.m_job = this;
        for (int i = 0; i < k_tokensPerJob; i++)
        {
            int err = xQueueSend(m_tokensSourceQueue, &myToken, portMAX_DELAY); 
            assert(err == pdTRUE);
        }
    }
    ~CvbsAnalyzerJob()
    {
        vQueueDelete(m_tokensSourceQueue);
        m_tokensSourceQueue = nullptr;
    }

    JobQueueToken TakeToken()
    {
        JobQueueToken token;
        int err = xQueueReceive(m_tokensSourceQueue, &token, portMAX_DELAY);
        assert(err == pdTRUE); 
        assert(token.m_job == this); // Ensure that the token belongs to this job
        return token;
    }

    void ReturnToken(JobQueueToken token)
    {
        int err = xQueueSend(m_tokensSourceQueue, &token, portMAX_DELAY);
        assert(err == pdTRUE); 
    }
    
    inline bool IsDone() const
    {
        const int tokensInSourceQueue = uxQueueMessagesWaiting(m_tokensSourceQueue);
        return (k_tokensPerJob == tokensInSourceQueue);
    }

    //Returns false if timeouted. timeoutMs = 0 means wait indefinitely.
    bool WaitUntilDone(uint32_t timeoutMs = 0) const
    {
        assert(k_tokensPerJob <= 1); // xQueuePeek is ok for 1, for more will need to wait until queue is full. this will need pop all tokens than push tme back after some count.)
        if (timeoutMs == 0)
        {
            JobQueueToken dummy;
            const bool notEmpty = xQueuePeek(m_tokensSourceQueue, &dummy, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
            return notEmpty;
        }
        else
        {
            JobQueueToken dummy;            
            const bool notEmpty = xQueuePeek(m_tokensSourceQueue, &dummy, portMAX_DELAY) == pdTRUE;
            return notEmpty;
        }
    }

    inline const Result GetResult() const
    {
        assert(IsDone()); 
        return m_result; //Return copy!
    }

    inline void SetResult(const Result& result) { 
        assert(!IsDone()); //When job is in done state, result no more changeable
        m_result = result;
    }    
    
    int m_gpioPin = -1;
    CvbsAnalyzerJobType m_type = CvbsAnalyzerJobType::k_none;
    CvbsAnalyzerInversionType m_inversionType = CvbsAnalyzerInversionType::k_nonInvertedOnly;
    QueueHandle_t m_tokensSourceQueue = nullptr;

private:
    Result m_result; 

    constexpr static size_t k_tokensPerJob = 1; 

};



#endif // CvbsAnalyzerJob_H