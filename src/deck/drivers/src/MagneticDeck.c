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
#include "task.h"
#include "log.h"
#include "param.h"

#define CONFIG_DEBUG = y

// ADC DMA configuration
#define ARRAY_SIZE 2048
uint16_t DMA_Buffer[ARRAY_SIZE];
ADC_TypeDef *ADC_n = ADC1;
DMA_Stream_TypeDef *DMA_Stream = DMA2_Stream4;
uint32_t DMA_Channel = DMA_Channel_0;
IRQn_Type DMA_IRQ = DMA2_Stream4_IRQn;
// 2^12 DOVE 12 SONO I BIT DELL'ADC DEL MCU = 4096
#define ADC_LEVELS 4096
#define ADC_MAX_VOLTAGE 3.3
#define PCLK2 84e6
#define ADC_PRESCALER 6
// 12 from bit and 15 from the register value 12+15 = 27
#define ADC_Full_Sampling_Time 27
#define Fc_ADC (PCLK2 / ADC_PRESCALER / ADC_Full_Sampling_Time)

// #define DMA_IRQ DMA2_Stream0_IRQn
uint8_t ADC_Channel = ADC_Channel_3;
volatile uint8_t test = 0;
static bool isInit = false;

// ADC flag to check if the conversion is done
volatile uint8_t ADC_Done = 0;

// FFT parameters
#define FFT_SIZE ARRAY_SIZE
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t fft_magnitude[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 fft_instance;
uint32_t fft_length = FFT_SIZE;
#define BIN_SIZE (Fc_ADC / FFT_SIZE)

// ------ Anchors Parameters -------
// Resonance Freqs Anchors in Hz
#define NeroResFreq 213e3
#define NeroIdx (int)(NeroResFreq / BIN_SIZE)
#define Nero_M -2.804
#define Nero_Q -2.635
int Nero_Position[] = {0, 0, 0};
#define Nero_Id 0

#define GialloResFreq 203e3
#define GialloIdx (int)(GialloResFreq / BIN_SIZE)
#define Giallo_M -2.887
#define Giallo_Q -2.629
int Giallo_Position[] = {1.99, 0.0, 0};
#define Giallo_Id 1

#define GrigioResFreq 193e3
#define GrigioIdx (int)(GrigioResFreq / BIN_SIZE)
#define Grigio_M -2.902
#define Grigio_Q -2.647
int Grigio_Position[] = {1.98, 1.97, 0};
#define Grigio_Id 2

#define RossoResFreq 183e3
#define RossoIdx (int)(RossoResFreq / BIN_SIZE)
#define Rosso_M -2.950
#define Rosso_Q -2.640
int Rosso_Position[] = {-0.05, 2.02, 0};
#define Rosso_Id 3

#define MagneticStandardDeviation 0.10

// -------  Debug variables -------
// ADC
volatile uint16_t firstValue = 0;
volatile uint16_t FirstVolt = 0;

// Distances
volatile float32_t Nero_distance = 0;
volatile float32_t Giallo_distance = 0;
volatile float32_t Grigio_distance = 0;
volatile float32_t Rosso_distance = 0;

// FFT
volatile uint16_t bin_size = BIN_SIZE;
volatile uint16_t Fc = Fc_ADC;
volatile uint16_t fft_size = FFT_SIZE;
volatile uint16_t Nero_IDX = NeroIdx; // Assign the constant value directly to the variable
volatile uint16_t Giallo_Idx = GialloIdx;
volatile uint16_t Grigio_Idx = GrigioIdx;
volatile uint16_t Rosso_Idx = RossoIdx;

// funzione che fa l'fft e prende in input il puntatore ad un buffer di float
void performFFT(float32_t *Input_buffer_pointer, float32_t *Output_buffer_pointer)
{
    // DEBUG_PRINT("performFFT started!\n");
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
    arm_mult_f32(Output_buffer_pointer, (float32_t *)flattop_2048_lut, Output_buffer_pointer, FFT_SIZE);

    // eseguo FFT
    arm_rfft_fast_f32(&fft_instance, Output_buffer_pointer, fft_output, 0);

    // normalizzo l'output
    // forse c'e una funzione per fare questo in automatico
    int i = 0;
    for (i = 0; i < FFT_SIZE; i++)
    {
        fft_output[i] = fft_output[i] / FFT_SIZE;
    }

    // calcolo le ampiezze della fft
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

    // arm_max_f32(&fft_magnitude[valueToExclude], (FFT_SIZE / 2) - valueToExclude, &maxval, &maxindex);
    // IDx = maxindex + valueToExclude;

    float32_t NeroAmpl = fft_magnitude[Nero_IDX];
    float32_t GialloAmpl = fft_magnitude[GialloIdx];
    float32_t GrigioAmpl = fft_magnitude[GrigioIdx];
    float32_t RossoAmpl = fft_magnitude[RossoIdx];

    // DEBUG_PRINT("NeroAmpl: %f\n", NeroAmpl);

    // compute the the distances from the amplitude of each anchor
    Nero_distance = pow(10, (log10(NeroAmpl) - Nero_Q) / Nero_M);
    Giallo_distance = pow(10, (log10(GialloAmpl) - Giallo_Q) / Giallo_M);
    Grigio_distance = pow(10, (log10(GrigioAmpl) - Grigio_Q) / Grigio_M);
    Rosso_distance = pow(10, (log10(RossoAmpl) - Rosso_Q) / Rosso_M);

    // if ((options->combinedAnchorPositionOk || options->anchorPosition[current_anchor].timestamp) &&
    //     (diff < (OUTLIER_TH * stddev)))
    // {
    // distanceMeasurement_t dist;
    // dist.distance = state.distance[current_anchor];
    // dist.x = options->anchorPosition[current_anchor].x;
    // dist.y = options->anchorPosition[current_anchor].y;
    // dist.z = options->anchorPosition[current_anchor].z;
    // dist.anchorId = current_anchor;
    // dist.stdDev = 0.25;
    // estimatorEnqueueDistance(&dist);

    // Nero
    distanceMeasurement_t dist_Nero;
    dist_Nero.distance = Nero_distance;
    dist_Nero.x = Nero_Position[0];
    dist_Nero.y = Nero_Position[1];
    dist_Nero.z = Nero_Position[2];
    dist_Nero.anchorId = Nero_Id;
    dist_Nero.stdDev = MagneticStandardDeviation;
    // DEBUG_PRINT("Nero Distance: %f\n", Nero_distance);
    estimatorEnqueueDistance(&dist_Nero);

    // // Giallo
    distanceMeasurement_t dist_Giallo;
    dist_Giallo.distance = Giallo_distance;
    dist_Giallo.x = Giallo_Position[0];
    dist_Giallo.y = Giallo_Position[1];
    dist_Giallo.z = Giallo_Position[2];
    dist_Giallo.anchorId = Giallo_Id;
    dist_Giallo.stdDev = MagneticStandardDeviation;
    estimatorEnqueueDistance(&dist_Giallo);

    // // Grigio
    distanceMeasurement_t dist_Grigio;
    dist_Grigio.distance = Grigio_distance;
    dist_Grigio.x = Grigio_Position[0];
    dist_Grigio.y = Grigio_Position[1];
    dist_Grigio.z = Grigio_Position[2];
    dist_Grigio.anchorId = Grigio_Id;
    dist_Grigio.stdDev = MagneticStandardDeviation;
    estimatorEnqueueDistance(&dist_Grigio);

    // // Rosso
    distanceMeasurement_t dist_Rosso;
    dist_Rosso.distance = Rosso_distance;
    dist_Rosso.x = Rosso_Position[0];
    dist_Rosso.y = Rosso_Position[1];
    dist_Rosso.z = Rosso_Position[2];
    dist_Rosso.anchorId = Rosso_Id;
    dist_Rosso.stdDev = MagneticStandardDeviation;
    estimatorEnqueueDistance(&dist_Rosso);
}

static void mytask(void *param)
{
    DEBUG_PRINT("Wait for system starting\n");
    systemWaitStart();
    DEBUG_PRINT("System Started\n");

    // gpio init
    GPIO_init(DECK_GPIO_RX2);
    // DMA init
    DMA_inititalization(RCC_AHB1Periph_DMA2, DMA_Stream, DMA_Buffer, ADC_n, DMA_Channel, DMA_IRQ, ARRAY_SIZE);
    // adc init
    ADC_init_DMA_mode(RCC_APB2Periph_ADC1, ADC_n);
    // Call the ADC_DMA_start function

    ADC_DMA_start(ADC_n, ADC_Channel, 1, ADC_SampleTime_15Cycles);

    arm_rfft_fast_init_f32(&fft_instance, fft_length);
    // DEBUG_PRINT("FFT initialized\n");
    while (1)
    {
        // DEBUG_PRINT("While\n");
        if (ADC_Done == 1)
        {

            performFFT(DMA_Buffer, fft_input);
            firstValue = DMA_Buffer[1000];
            FirstVolt = firstValue * ADC_MAX_VOLTAGE / ADC_LEVELS;

            DMA_inititalization(RCC_AHB1Periph_DMA2, DMA_Stream, DMA_Buffer, ADC_n, DMA_Channel, DMA_IRQ, ARRAY_SIZE);
            ADC_init_DMA_mode(RCC_APB2Periph_ADC1, ADC_n);
            ADC_DMA_start(ADC_n, ADC_Channel, 1, ADC_SampleTime_15Cycles);

            ADC_Done = 0;
            // DEBUG_PRINT("First Value: %d\n", firstValue);
        }
        else
        {
            // DEBUG_PRINT("ADC_Done is 0\n");
        }

        vTaskDelay(M2T(100));
    }
}

void DMA2_Stream4_IRQHandler(void)
{
    // DEBUG_PRINT("DMA2_Stream4_IRQHandler\n");
    // if (DMA_GetITStatus(DMA_Stream, DMA_IT_HTIF0)) //&& (xSemaphoreTake(semaphoreHalfBuffer, 0) == pdTRUE))
    // {
    //     // ADC_Cmd(ADC_n, DISABLE);
    //     // DEBUG_PRINT("ADC disabled\n");
    //     DMA_ClearITPendingBit(DMA_Stream, DMA_IT_HTIF0);
    // }

    if (DMA_GetITStatus(DMA_Stream, DMA_IT_TCIF4)) //&& (xSemaphoreTake(semaphoreHalfBuffer, 0) == pdTRUE))
    {
        // ADC_Cmd(ADC_n, DISABLE);
        // DEBUG_PRINT("ADC disabled\n");
        DMA_ClearITPendingBit(DMA_Stream, DMA_IT_TCIF4);
        ADC_Done = 1;
    }
}

static void magneticInit()
{
    if (isInit)
    {
        return;
    }
    DEBUG_PRINT("MAGNETIC init started!\n");

    xTaskCreate(mytask, MAGNETIC_TASK_NAME,
                MAGNETIC_TASK_STACKSIZE, NULL, MAGNETIC_TASK_PRI, NULL);

    // DEBUG_PRINT("MAGNETIC init ended!\n");
    isInit = true;
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
};

DECK_DRIVER(magneticDriver);

#define CONFIG_DEBUG_LOG_ENABLE = y

// LOG_GROUP_START(ADC)
// LOG_ADD_DEBUG(LOG_UINT16, firstValue, &firstValue)
// LOG_ADD_DEBUG(LOG_UINT16, FirstVolt, &FirstVolt)
// LOG_GROUP_STOP(ADC)

LOG_GROUP_START(MAGNETIC_DISTANCES)
LOG_ADD_CORE(LOG_FLOAT, Nero, &Nero_distance)
LOG_ADD_CORE(LOG_FLOAT, Giallo, &Giallo_distance)
LOG_ADD_CORE(LOG_FLOAT, Grigio, &Grigio_distance)
LOG_ADD_CORE(LOG_FLOAT, Rosso, &Rosso_distance)
LOG_GROUP_STOP(MAGNETIC_DISTANCES)

PARAM_GROUP_START(FFT_Param)
// PARAM_ADD_CORE(PARAM_UINT16, NeroResFreq, &NeroResFreq)
// volatile int Nero_IDX = NeroIdx;
PARAM_ADD_CORE(PARAM_UINT16, Nero_Index, &Nero_IDX)
PARAM_ADD_CORE(PARAM_UINT16, Giallo_Index, &Giallo_Idx)
PARAM_ADD_CORE(PARAM_UINT16, Grigio_Index, &Grigio_Idx)
PARAM_ADD_CORE(PARAM_UINT16, Rosso_Index, &Rosso_Idx)
PARAM_GROUP_STOP(FFT_Param)