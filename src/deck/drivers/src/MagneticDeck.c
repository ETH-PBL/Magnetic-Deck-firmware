#define DEBUG_MODULE "MagneticDeck"
#include "debug.h"
#include "deck.h"
#include "deck_analog.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx_misc.h"
#include "timers.h"
#include "stm32f4xx_tim.h"
#include "arm_math.h"
#include "FlattopWinFromPython.h"
#include "arm_const_structs.h"

#define CONFIG_DEBUG = y

// ADC DMA configuration
#define ARRAY_SIZE 4096
uint16_t DMA_Buffer[ARRAY_SIZE];
float32_t adc_buf_float[ARRAY_SIZE];
ADC_TypeDef *ADC_n = ADC1;
DMA_Stream_TypeDef *DMA_Stream = DMA2_Stream4;
uint32_t DMA_Channel = DMA_Channel_0;
IRQn_Type DMA_IRQ = DMA2_Stream4_IRQn;
uint8_t ADC_Channel = ADC_Channel_7;
volatile uint8_t test = 0;

// Timer configuration
#define TIM_PERIF RCC_APB1Periph_TIM5
#define TIM TIM5
#define TIM_DBG DBGMCU_TIM5_STOP
#define TIM_IRQn TIM5_IRQn
IRQn_Type TIM_IRQ = TIM5_IRQn;
#define TIM_PERIOD 0x40

// FFT parameters
#define FFT_SIZE ARRAY_SIZE / 2
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t fft_magnitude[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 fft_instance;
uint32_t fft_length = FFT_SIZE;

// 2^12 DOVE 12 SONO I BIT DELL'ADC DEL MCU = 4096
#define ADC_LEVELS 4096
#define ADC_MAX_VOLTAGE 3.3
// samplingFreq
#define samplingFrequency 300000
#define numbersOfSamples 4096
// Resonance Freqs Anchors in Hz
#define NeroResFreq 117000
#define NeroIdx 1598
#define GialloResFreq 97000
#define GialloIdx 1325
#define GrigioResFreq 107000
#define GrigioIdx 1462
#define RossoResFreq 87000
#define RossoIdx 1189

// Create the semaphores
static SemaphoreHandle_t semaphoreHalfBuffer;
static SemaphoreHandle_t semaphoreFullBuffer;

// funzione che fa l'fft e prende in input il puntatore ad un buffer di float
void performFFT(float32_t *Input_buffer_pointer, float32_t *Output_buffer_pointer)
{

    // perform FFT on first half of buffer
    arm_q15_to_float((q15_t *)Input_buffer_pointer, Output_buffer_pointer, FFT_SIZE);
    int p = 0;
    for (p = 0; p < FFT_SIZE; p++)
    {
        // Fattore di scala della conversione della funzione, guarda doc per capire
        Output_buffer_pointer[p] = Output_buffer_pointer[p] * 32768;

        // ADCLevelsToVolt
        Output_buffer_pointer[p] = Output_buffer_pointer[p] * (ADC_MAX_VOLTAGE / ADC_LEVELS);
    }
    // applico la finestratura flattop
    arm_mult_f32(Output_buffer_pointer, (float32_t *)FlattopWinFromPython, Output_buffer_pointer, FFT_SIZE);

    // eseguo FFT
    arm_rfft_fast_f32(&fft_instance, Output_buffer_pointer, fft_output, 0);

    // normalizzo l'output
    int i = 0;
    for (i = 0; i < FFT_SIZE; i++)
    {
        fft_output[i] = fft_output[i] / FFT_SIZE;
    }

    // calcolo le ampiezze della fft
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

    // leggo le ampiezze per ciascun ancora

    // float32_t NeroAmpl = fft_magnitude[NeroIdx];
    // float32_t GialloAmpl = fft_magnitude[GialloIdx];
    // float32_t GrigioAmpl = fft_magnitude[GrigioIdx];
    // float32_t RossoAmpl = fft_magnitude[RossoIdx];

    // TODO CAPIRE QUI COME MANDARE I DATI

    // if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
    //     (diff < (OUTLIER_TH * stddev)))
    // {
    //     distanceMeasurement_t dist;
    //     dist.distance = state.distance[current_anchor];
    //     dist.x = options->anchorPosition[current_anchor].x;
    //     dist.y = options->anchorPosition[current_anchor].y;
    //     dist.z = options->anchorPosition[current_anchor].z;
    //     dist.anchorId = current_anchor;
    //     dist.stdDev = 0.25;
    //     estimatorEnqueueDistance(&dist);
    // }
}

void TIM5_IRQHandler(void)
{
    if (test == 50)
    {
        test = 46;
        if (xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY) == pdTRUE)
        {
            test = 1;
            // uint16_t *firstHalfPointer = &DMA_Buffer[0];
            // performFFT(DMA_Buffer, adc_buf_float);
        }
        else if (xSemaphoreTake(semaphoreFullBuffer, portMAX_DELAY) == pdTRUE)
        {
            test = 1;
            // perform FFT on second half of buffer
            // performFFT(DMA_Buffer + FFT_SIZE, adc_buf_float + FFT_SIZE);
        }
    }

    test = 2;
    // if (TIM_GetITStatus(TIM, TIM_IT_Trigger))
    // {
    //     test = 98;
    //     TIM_ClearITPendingBit(TIM, TIM_IT_Trigger);
    // }

    // if (TIM_GetITStatus(TIM, TIM_IT_Update))
    // {
    //     test = 99;
    //     TIM_ClearITPendingBit(TIM, TIM_IT_Update);
    //     if (xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY) == pdTRUE)
    //     {
    //         test = 1;
    //         // uint16_t *firstHalfPointer = &DMA_Buffer[0];
    //         performFFT(DMA_Buffer, adc_buf_float);
    //     }
    //     else if (xSemaphoreTake(semaphoreFullBuffer, portMAX_DELAY) == pdTRUE)
    //     {
    //         test = 1;
    //         // perform FFT on second half of buffer
    //         performFFT(DMA_Buffer + FFT_SIZE, adc_buf_float + FFT_SIZE);
    //     }
    // }
}

void DMA2_Stream4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA_Stream, DMA_IT_HTIF4) && (xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY) == pdTRUE))
    {
        test = 50;
        // Give the semaphore
        xSemaphoreGive(semaphoreHalfBuffer);
        DMA_ClearITPendingBit(DMA_Stream, DMA_IT_HTIF4);
    }
    if (DMA_GetITStatus(DMA_Stream, DMA_IT_TCIF4) && (xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY) == pdTRUE))
    {
        test = 50;
        // xgetTick();
        // Give the semaphore
        xSemaphoreGive(semaphoreFullBuffer);
        DMA_ClearITPendingBit(DMA_Stream, DMA_IT_TCIF4);
    }
}

static void setUpHwTimer()
{
    // Init structures
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(TIM_PERIF, ENABLE);

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Timer configuration
    TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0x05;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);

    // Enable the timer
    TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);

    // Enable the timer
    TIM_Cmd(TIM, ENABLE);
}

static void magneticInit()
{
    DEBUG_PRINT("MAGNETIC init started!\n");
    // Set up the semaphores
    semaphoreHalfBuffer = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphoreHalfBuffer);
    semaphoreFullBuffer = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphoreFullBuffer);
    DEBUG_PRINT("MAGNETIC Semaphore created!\n");

    // xSemaphoreGive(semaphoreHalfBuffer);
    // if (xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY) == pdTRUE)
    // {
    //     test = 1;
    // }
    // else
    // {
    //     test = 2;
    // }
    // setup the hardware timer
    // setUpHwTimer();
    // setup the FFT instance
    arm_rfft_fast_init_f32(&fft_instance, fft_length);

    // RCC_APB2_Peripherals RCC_APB2_Peripheral = RCC_APB2Periph_ADC1;
    // gpio init
    GPIO_init(GPIO_PinSource3);
    // DMA init
    DMA_inititalization(RCC_AHB1Periph_DMA2, DMA_Stream, DMA_Buffer, ADC_n, DMA_Channel, DMA_IRQ, ARRAY_SIZE);
    // adc init
    ADC_init_DMA_mode(RCC_APB2Periph_ADC1, ADC_n);
    // dma start
    // adc start

    // // Take both the buffers
    // xSemaphoreTake(semaphoreHalfBuffer, portMAX_DELAY);
    // xSemaphoreTake(semaphoreFullBuffer, portMAX_DELAY);

    // Call the ADC_DMA_start function
    ADC_DMA_start(ADC_n, ADC_Channel, 1, ADC_SampleTime_15Cycles);

    DEBUG_PRINT("MAGNETIC init ended!\n");
}

static bool magneticTest()
{
    DEBUG_PRINT("MAGNETIC test passed!\n");
    return true;
}

static const DeckDriver magneticDriver = {
    .name = "Magnetic",
    .usedGpio = DECK_USING_PA3,
    .init = magneticInit,
    .test = magneticTest,
    .usedPeriph = DECK_USING_TIMER5,
};

DECK_DRIVER(magneticDriver);