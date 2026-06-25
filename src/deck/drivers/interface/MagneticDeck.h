#ifndef __MAGNETIC_DECK_H__
#define __MAGNETIC_DECK_H__

#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "arm_math.h"
#include "deck_analog.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#define MODEL_TO_USE 0 // 0 for Nelder-Mead strictly, 1 for Nelder-Mead with Kalman Filter
#define Z_MEASUREMENTS_TO_ACCUMULATE 3
#define TARGET_FLYING_HEIGHT 0.20f

// -------------------------- ADC -------------------------------------------
// ADC DMA configuration
#define ARRAY_SIZE 2048

// 2^12 WHERE 12 IS THE NUMBER OF BITS OF THE MCU ADC = 4096
#define ADC_LEVELS 4096
#define ADC_MAX_VOLTAGE 3.0f
#define PCLK2 84e6f
#define ADC_PRESCALER 6.0f
// 12 from bit and 15 from the register value 12+15 = 27
#define ADC_Full_Sampling_Time 27.0f
#define Fc_ADC (PCLK2 / ADC_PRESCALER / ADC_Full_Sampling_Time) // 518 KHz

#define ADC_Channel_Default ADC_Channel_3;
// -------------------------- DMA -------------------------------------------
#define DMA_IRQ DMA2_Stream4_IRQn
#define MY_DMA_Channel DMA_Channel_0
#define MY_DMA_Stream DMA2_Stream4

// IRQn_Type DMA_IRQ = DMA2_Stream4_IRQn;

// -------------------------- FFT -------------------------------------------
#define FFT_SIZE ARRAY_SIZE
#define BIN_SIZE (int)(Fc_ADC / FFT_SIZE) // 253 Hz
#define SATURATION_TRESHOLD 1.2f

// ------------------------ Measurement Model Params -------------------------------------------
#define Default_MagneticStandardDeviation 0.0001f
#define G_INA 100.0f
#define Optimization_Model_STD 0.06f
static float Optimization_Model_STD_Z = 0.15f;

#define offsetCoil 0.025f // 3 cmf
#define robodogOffset 0.33f

// ------------------------ Linear Kalman Filter Params -------------------------------------------
#define STATE_DIM 3   // Stato: [x, y, z]
#define MEASURE_DIM 3 // Misure: [x, y, z]
#define EPSILON 1e-6f // Termine di regolarizzazione

typedef struct
{
    float F[STATE_DIM][STATE_DIM];
    float H[MEASURE_DIM][STATE_DIM];
    float Q[STATE_DIM][STATE_DIM];
    float R[MEASURE_DIM][MEASURE_DIM];
    float P[STATE_DIM][STATE_DIM];
    float x_state[STATE_DIM];
    float z_meas[MEASURE_DIM];
} KalmanFilter;

#define q_kf_default 0.025f
#define sigma_x_kf_default 0.05f // will be squared in the R matrix

// const float q = 0.05f;
// const float sigma_x = 0.05f * 0.05f;

// -------------------------  adaptive std on Measured Voltage -------------------------------
#define UseAdaptiveSTD 0
#define window_size 25

// ------------------------ Calibration -------------------------------------------
#define CALIBRATION_TIC_VALUE 500.0f // Number of measurements to perform the calibration

// ------------------------System HZ-------------------------------------------
#define SYSTEM_HZ 30
#define SYSTEM_PERIOD_MS (1000 / SYSTEM_HZ)

// ------------------------ Anchors Parameters -------------------------------------------
#define NUM_ANCHORS 4

// // NERO COIL
// // Resonance Freqs Anchors in Hz
// // #define Coil_1ResFreq 213e3
// #define Coil_3ResFreq 210e3
// #define Coil_3Idx (int)(Coil_3ResFreq / BIN_SIZE)
// #define Coil_3_M -2.804
// #define Coil_3_Q -2.635
// #define Coil_3_Position_x +0.245f
// #define Coil_3_Position_y +0.3325f
// #define Coil_3_Position_z +0.25f
// #define Coil_3_Id 0

// // GIALLO COIL
// // #define Coil_2ResFreq 203e3
// #define Coil_4ResFreq 199e3
// #define Coil_4Idx (int)(Coil_4ResFreq / BIN_SIZE)
// #define Coil_4_M -2.887
// #define Coil_4_Q -2.629
// #define Coil_4_Position_x +0.255f
// #define Coil_4_Position_y -0.3325f
// #define Coil_4_Position_z +0.25f
// #define Coil_4_Id 1

// // GRIGIO COIL
// // #define Coil_3ResFreq 193e3
// #define Coil_2ResFreq 189e3 // 189 default mcu
// #define Coil_2Idx (int)(Coil_2ResFreq / BIN_SIZE) // index num 747
// #define Coil_2_M -2.902
// #define Coil_2_Q -2.647
// #define Coil_2_Position_x -0.255f
// #define Coil_2_Position_y -0.3325f
// #define Coil_2_Position_z +0.25f
// #define Coil_2_Id 2

// // ROSSO COIL
// // #define Coil_4ResFreq 183e3
// #define Coil_1ResFreq 181e3
// #define Coil_1Idx (int)(Coil_1ResFreq / BIN_SIZE)
// #define Coil_1_M -2.950
// #define Coil_1_Q -2.640
// #define Coil_1_Position_x -0.245f
// #define Coil_1_Position_y +0.3325f
// #define Coil_1_Position_z +0.25f
// #define Coil_1_Id 3

// NERO COIL
// Resonance Freqs Anchors in Hz
// #define Coil_1ResFreq 213e3
#define Coil_3ResFreq 210e3
#define Coil_3Idx (int)(Coil_3ResFreq / BIN_SIZE)
#define Coil_3_M -2.804
#define Coil_3_Q -2.635
#define Coil_3_Position_x +0.295f
#define Coil_3_Position_y +0.25f
#define Coil_3_Position_z +0.25f
#define Coil_3_Id 0

// GIALLO COIL
// #define Coil_2ResFreq 203e3
#define Coil_4ResFreq 199e3
#define Coil_4Idx (int)(Coil_4ResFreq / BIN_SIZE)
#define Coil_4_M -2.887
#define Coil_4_Q -2.629
#define Coil_4_Position_x +0.295f
#define Coil_4_Position_y -0.25f
#define Coil_4_Position_z +0.25f
#define Coil_4_Id 1

// GRIGIO COIL
// #define Coil_3ResFreq 193e3
#define Coil_2ResFreq 189e3                       // 189 default mcu
#define Coil_2Idx (int)(Coil_2ResFreq / BIN_SIZE) // index num 747
#define Coil_2_M -2.902
#define Coil_2_Q -2.647
#define Coil_2_Position_x -0.295f
#define Coil_2_Position_y -0.25f
#define Coil_2_Position_z +0.25f
#define Coil_2_Id 2

// ROSSO COIL
// #define Coil_4ResFreq 183e3
#define Coil_1ResFreq 181e3
#define Coil_1Idx (int)(Coil_1ResFreq / BIN_SIZE)
#define Coil_1_M -2.950
#define Coil_1_Q -2.640
#define Coil_1_Position_x -0.295f
#define Coil_1_Position_y +0.25f
#define Coil_1_Position_z +0.25f
#define Coil_1_Id 3

// ------------------------ PHYSICAL COIL -------------------------------------------
#define RAY 0.019f
#define N_WOUNDS 5.0f
#define COIL_SURFACE (RAY * RAY * PI)
#define CURRENT 0.5f // This maybe can be improved
#define MU_0 1.25663706212e-06f

// ------------------------ Gain -------------------------------------------
// Potentiometer Params
#define R10 200.0f  // 200 Ohm
#define RW_2_7V 155 // ohm  155 OHM TYPICAL_DC_WIPER_RESISTANCE
#define RW_5_5V 100 // ohm

#define POTENTIOMETER_ADDR 0x2F // 0x94 WRITE 0x95 READ
#define POTENTIOMETER_BIT 7
// #define POTENTIOMETER_STEPS pow(2, POTENTIOMETER_BIT) // 7 bit --> 128
#define POTENTIOMETER_NUMBER_OF_STEPS (1 << POTENTIOMETER_BIT) - 1 // 127
#define POTENTIOMETER_FULL_SCALE_RAB 50E3                          // 50 kOhm
#define POTENTIOMETER_ANALOG_HW_DELAY_AFTER_SET 10
// in serie con l'INa c'e un operazionale che produce un guadagno di 10
#define DefaultPotentiometerValue G_INA / 10.0f
#define OpAmpGainValue 10.0f

// -------------------------- DAC -------------------------------------------
// DAC Reference Voltage Params
#define V_REF_CRAZYFLIE 3.0f
#define DAC_BIT 12
#define DAC_LEVELS (1 << DAC_BIT) // 4096
#define DAC_STEP (VREF / DAC_LEVELS)
#define DECK_DAC_I2C_ADDRESS 0x4C //
#define DAC_WRITE_LENGTH 2
#define DAC_ANALOG_HW_DELAY_AFTER_SET 10
#define V_DD 3.3f

// -------------------------- NELDER-MEAD structs -------------------------------------------
#define NM_OPTIMIZER_IMPLEMENTATION
#define NM_NO_DEBUG_LOG

// ========================== Function Definitions ==========================

// -------------------------- Magnetic Deck -------------------------------------------

uint16_t ValueforDAC_from_DesideredVolt(float desidered_Voltage);

uint16_t WiperResistanceValue_From_Interpolation(float Vdd);

float Potentiometer_Resistance_Value_from_Desidered_Gain(float desidered_Gain);

uint8_t Potentiometer_Value_To_Set(float desidered_Gain);

void DMA2_Stream4_IRQHandler(void);

void ADC1_IRQHandler(void);
typedef struct
{
    float Coil_1Ampl;
    float Coil_2Ampl;
    float Coil_3Ampl;
    float Coil_4Ampl;
    float AllAmpl[NUM_ANCHORS];
} FFT_Amplitudes;

FFT_Amplitudes performFFT(uint32_t *Input_buffer_pointer, float32_t *Output_buffer_pointer, float32_t flattopCorrectionFactor);

void check_saturations(FFT_Amplitudes *amplitudes, int *Id_in_saturation, bool *there_is_saturation);

void finalizeCycle();

// -------------------------- Measurement Model  -------------------------------------------

void get_B_field_for_a_Anchor(float *anchor_pos,
                              const float *tag_pos,
                              float *tag_or_versor,
                              float *B_field);

float V_from_B(float *B_field, float *rx_versor, float resonanceFreq, float Gain);

// -------------------------- NELDER-MEAD Functions -------------------------------------------
const float myCostFunction_3A(int n, const float *x, void *arg);
const float myCostFunction_4A(int n, const float *x, void *arg);

// -------------------------- Math Utils -------------------------------------------

float dot_product(float *a, float *b, int length);

float euclidean_distance(float *a, float *b, int length);

void getversor(float *a, float *b, float *u, int length);

float computeSTD(float *data, int arrayDimension);
#define SQUARE(x) ((x) * (x))

#endif // __MAGNETIC_DECK_H__