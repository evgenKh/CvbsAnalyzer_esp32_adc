# Код для оцінки сигналу який поступає на АЦП esp32, чи є він аналоговим відео, чи ні.

## Підтримка версій IDE IDE:
- Platformio+VSCode:
    - Використовується старий драйвер ADC, ще без continuous mode, через I2S. (файли FastADC.cpp/h)
    - Platform espressif32 @ 6.10.0
    - framework-arduinoespressif32 @ 3.20017.241212+sha.dcc1105b (https://github.com/platformio/platform-espressif32)
        - Contains Arduino support - v2.0.17 (based on IDF v4.4.7)
        - Contains ESP-IDF support(without arduino) - v5.4.0
- Arduino IDE 2.3.6
    - esp32 пакет 3.2.0 новий драйвер adc_continuous (файли FastADCContinuous.cpp/h)
	- esp32 пакет 3.0.0 ймовірно теж але треба перевірити
	- також працює в не-continuous режимі якщо в arduino IDE завантажений пакет esp версії **2.0.17** і в ньому руками заглушен варнінг збірки в \AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.17\tools\sdk\esp32\include\hal\esp32\include\hal\i2s_ll.h:766
		
		![arduino_esp32_v2.0.17.png](arduino_esp32_v2.0.17.png)	
		
		```
		static inline void i2s_ll_rx_get_pdm_dsr(i2s_dev_t *hw, i2s_pdm_dsr_t *dsr)
		{
			//to silence build error
			//*dsr = hw->pdm_conf.rx_sinc_dsr_16_en;
			*dsr = (i2s_pdm_dsr_t)hw->pdm_conf.rx_sinc_dsr_16_en;
		}
		```

## Підключення: 

- Тільки ADC1 входи!

- Відеосигнал має бути DC-coupled, тобто завжди >0вольт відносно землі, навіть підчас синхроімпульса!
Для цьго можна задіяти просту схему:
![image](Схема_підключення.jpg)

## Алгоритм:

- З частотою 1-2МГц зчитуємо напругу з АЦП. Масивами по 400-1000 семплів.
- в PreProcessBuf відкидаємо 4 біта з 16-бітної змінної, бо роздільна здатність АЦП 12біт. Тут же за потреби інвертуємо по формулі y=4095-x
- AmplitudeCalculator накопичує бажану кількість семплів в:      
    - Cвою гістограму напруги m_amplitudeHistogram.
    - Паралельно ще в свій масив m_flatnessAccumulator, де накопичує значення `y=-log2(x')`, групируючи за значенням напруги зчитанного з ADC. Тобто для значення напруги яке найчастіше було частиною прямої лінії на графіку(супроводжувалось низькими значеннями диференціала), бути мати найбільший бал в m_flatnessAccumulator. Диференціал рахується просто як abs(sample[n]-sample[n-1]).
    - (з відкиданням усіх 0 та 4095 значень які йдуть на початку масиву. Через хаки якими налаштовуємо АЦп вже після початку семплінгу а потім робимо zeroDma, неактуальні дані можуть бути затерті нулями) (З пропуском семплів черрез один, дивись k_adcDataStrideSamples. Теж наслідок кривого драйверу АЦП, експериментально визначено з так найменш шумні дані).
- AmplitudeCalculator проводить пошук погорового значення синхроімпульса трьома способами:
    - В m_flatnessAccumulator шукає саму ліву послідовність risingEdge->fallingEdge->risingEdge. Середина між початковими точками другого та третього edge - визначається як syncTreshold
    ![m_flatnessAccumulator](m_flatnessAccumulator.png)
    - Якщо в m_flatnessAccumulator дані погані(недостаньо семплів або їх амплітуди), такий самий робиться натомість на гістограмі напруги m_amplitudeHistogram    
    ![m_amplitudeHistogram](m_amplitudeHistogram.png)
    - На випадок поганих значень в обох масивах, останній спосіб: syncTreshold = 15% від размаху між мін. і макс. значеннями семплів.
- Коли порогове значення визначено, далі пушимо семпли в SyncIntervalsCalculator. Який одразу робить такі операції: знаходить інтервали коли напруга відносить до sync або до notSync. Довжину інтервалів не зберігає, а одразу додає до одної з двох гістограм, де зберігається кількість скільки зустрілось інтервалів довжиною в N мікросекунд(або семплів). не повні інтервали на початку/кінці запису відкидаються.
![sync_notSync_intervals](sync_notSync_intervals.png)
- Коли накопичена деяка кількість цієї статистики, дві гістограми передаються в клас VideoScore для ітогової оцінки. 
Приклади гістограм: 
![sync_cnotSync_histograms](sync_cnotSync_histograms.png)

- Виділяємо цікаві нам інтервали на гістограмах, такі де при наявності відео очікується нормальне розподілення(лише з одним піком! Опрацювати багато піків мені не вдалось).
    ![SequenceHist_no_video.png](SequenceHist_no_video.png)
    ![SequenceHist_with_video.png](SequenceHist_with_video.png)

    - На кожному відрізку гістограми який нас цікавить, рахуємо матожиданіє(weighted mean) i стандартне відхилення.
    - Оцінюємо наскільки воно схоже на відеосигнал по наступним формулам:
    
|syncMeanScore|syncStDevScore|notSyncMeanScore|notSyncStDev|
|---|---|---|---|
| ![syncMeanScore.png](syncMeanScore.png) | ![syncStDevScore.png](syncStDevScore.png) | ![notSyncMeanScore.png](notSyncMeanScore.png) | ![notSyncStDev.png](notSyncStDev.png) |
- 
    - Об'єднуємо ці 4 оцінки в одну загальну наприклад по такій формулі: `float totalScore = ((syncMeanScore * syncStDevScore)* 0.6f + (notSyncMeanScore * notSyncStDevScore)) / (0.6f + 1.0f);`
    - Наверняка робиться зайва робота спочатку по знаходженню статистичних метрик, потім їх інерполяція і оцінювання і всяке таке. І навіть все робиться у float. Але працює на диво швидко.

- Більш простий спосіб який зараз не задіяний:
    (VideoScore::CalculateFromSyncIntervalsSimple)
    Ітогова оцінка складається з:
        - на 70% з не-синхро інтервалів довжиною 57-63us (макс оцінка якщо до них відносяться 80% інтервалів)
        - на 20% з синхро-інтервалів 5-7us (макс оцінка якщо до них відносяться 60% інтервалів)
        - на 5% з не-синхро інтервалів довжиною 29-33us (макс оцінка якщо до них відносяться 20% інтервалів)
        - на 5% з не-синхро інтервалів довжиною 5-7us (макс оцінка якщо до них відносяться 20% інтервалів)
- В ідеалі усі ці коефіціенти мав би підбирати ШІ, але мені не вдалось його нормально задіяти, тож підібрав такі. Кожен з елементів алгоритма є state-machine, усі датасети друкуються в форматі CSV, тому мало б бути хоч всю систему повністю перетворити в нейронку.
- Якщо будь-який з компонентів зафейлився, наприклад недостатньо семплів, не вдалось знайти порогове значення, і т.д. то оцінка одразу виставляється 0.0

## Швидкодія:

Наразі це 8-13мс на прохід одного канала АЦП (з інверсієй + без, разом).
```
Video
 "m_stateProfilers": {
        "k_startADC": 1367,
        "k_samplingAndPreFiltering": 9632,
        "k_amplitudeSampling": 11,
        "k_restartInverted": 22,
        "k_stopADC": 52,
        "k_totalAnalyzeTime": 11115,
},
RSSI
"m_stateProfilers": {
        "k_startADC": 1219,
        "k_samplingAndPreFiltering": 431,
        "k_averageCalculation": 2195,
        "k_stopADC": 36,
        "k_totalAnalyzeTime": 3513,
},
```

Можлива асинхронна робота, приклад:
```
CvbsAnalyzer g_cvbsAnalyzer;
CvbsAnalyzerDispatcher g_cvbsAnalyzerDispatcher(&g_cvbsAnalyzer);
CvbsAnalyzerJob g_pin35Job(CvbsAnalyzerJobType::k_videoScore, 35);
CvbsAnalyzerJob g_pin36Job(CvbsAnalyzerJobType::k_videoScore, 36);
CvbsAnalyzerJob g_pin36AverageJob(36, CvbsAnalyzerJobType::k_averageRssi, CvbsAnalyzerInversionType::k_nonInvertedOnly);

g_cvbsAnalyzer.InitializeFastADC();
g_cvbsAnalyzerDispatcher.StartWorkerThread();

while(1)
{
	g_cvbsAnalyzerDispatcher.RequestJob(&g_pin35Job);
	g_cvbsAnalyzerDispatcher.RequestJob(&g_pin36Job);
	g_pin35Job.WaitUntilDone();
	CVBS_ANALYZER_LOG_INFO("m_videoScore.m_isVideo=%f\n", g_pin35Job.m_videoScore.m_isVideo);

	//або

	while(!g_pin36AverageJob.IsDone()) {}
	CVBS_ANALYZER_LOG_INFO("m_rssiAverage=%d\n", g_pin36AverageJob.m_rssiAverage);
}
```



## Приклад результату роботи

- На пін 36 підключено якісне не-інвертоване відео. Пін 35 висить в повітрі:
    ```
    Reading pin 36   
    //non inverted  
    syncHistMeanSamples=5.000000 syncHistStDeviation=0.000000 
    notSyncHistMeanSamples=59.000000 notSyncHistStDeviation=0.000000
    //inverted
    syncHistMeanSamples=1.777778 syncHistStDeviation=11.195237 notSyncHistMeanSamples=0.000000 notSyncHistStDeviation=0.000000

    m_videoScore.m_isVideo=0.994267 m_videoScoreInverted.m_isVideo=0.000000

	Reading pin avg 36              : m_rssiAverage=3908

	Reading pin 35          : m_videoScore.m_isVideo=0.000000 m_videoScoreInverted.m_isVideo=0.000000
	```


Необроблені дані з platformio зі старим драйвером, і arduino з новим:


![continuous_non-continuous_freq_drift.png](continuous_non-continuous_freq_drift.png)