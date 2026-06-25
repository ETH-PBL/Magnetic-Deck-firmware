#define DEBUG_MODULE "MagneticDeck"
#include "log.h"
#include "debug.h"
#include "param.h"
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
#include "system.h"
#include "param.h"
#include "i2cdev.h"
#include "MagneticDeck.h"
#include "nelder_mead_3A.h"
#include "nelder_mead_4A.h"
#include "usec_time.h"
#include <time.h>
#include <stdlib.h>

#include "nelder_mead_4A.h"
#include "nelder_mead_3A.h"
#include "estimator_kalman.h"
#include "estimator.h"
#include "stabilizer_types.h"
#include "kalman_core.h"
#include "LinearKalmanFilterRW.h"

// #define CONFIG_DEBUG = y
static bool isInit = false;

// Anchors
const float AnchorPositionMatrix[4][3] = {
    {Coil_1_Position_x, Coil_1_Position_y, Coil_1_Position_z},
    {Coil_2_Position_x, Coil_2_Position_y, Coil_2_Position_z},
    {Coil_3_Position_x, Coil_3_Position_y, Coil_3_Position_z},
    {Coil_4_Position_x, Coil_4_Position_y, Coil_4_Position_z}};

const float ResonanceFreqs[4] = {Coil_1ResFreq, Coil_2ResFreq, Coil_3ResFreq, Coil_4ResFreq};

// adc INITIALIZATION
// ADC flag to check if the conversion is done
uint8_t ADC_Done = 0;
static ADC_TypeDef *ADC_n = ADC1;
static uint8_t ADC_Channel = ADC_Channel_Default;

// DMA Initialization
static uint32_t DMA_Buffer[ARRAY_SIZE];
static DMA_Stream_TypeDef *DMA_Stream = MY_DMA_Stream;
static uint32_t DMA_Channel = MY_DMA_Channel;

// FFT parameters
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t fft_magnitude[FFT_SIZE / 2];
static arm_rfft_fast_instance_f32 fft_instance;
static uint16_t fft_length = FFT_SIZE;

// Potentiometer Params
float GainValue_Setted = 0;
float GainValue = DefaultPotentiometerValue;
// Total Gain
static float TotalGain = 0;

float MagneticStandardDeviation = Default_MagneticStandardDeviation;

// -------------------------- NelderMead -------------------------------------------
// Set the range where to look for the minumum
float range[3] = {+0.05f, +0.05f, 0.05f};
static float solution[3];
static float z_final_measurement = 0.0f;
static bool initializationOnOrigin = false;

// --------------------------Linear Kalman Filter-------------------------------------------
KalmanFilter kf;

// -------  Debug variables -------
// static int counterSaturation = 0;
static int counter = 0;

volatile float MeasuredVoltages_calibrated[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float Coil_1Ampl = 0;
float Coil_2Ampl = 0;
float Coil_3Ampl = 0;
float Coil_4Ampl = 0;

volatile float LKF_ESTIMATION_DEBUG[3] = {0.0f, 0.0f, 0.0f};

volatile float outlier = 0;
uint32_t currentCalibrationTick = 0;

// FFT
// static uint16_t bin_size = BIN_SIZE;
// static uint16_t fft_size = FFT_SIZE;
// static uint16_t Coil_1_Idx = Coil_1Idx;
// static uint16_t Coil_2_Idx = Coil_2Idx;
// static uint16_t Coil_3_Idx = Coil_3Idx;
// static uint16_t Coil_4_Idx = Coil_4Idx;

float skipThisMeasurement = 0;

// -------------------- DAC Functions -------------------------------
uint16_t ValueforDAC_from_DesideredVolt(float desidered_Voltage)
{
    // DAC7571
    // https://www.ti.com/lit/ds/symlink/dac7571.pdf?ts=1718008558964&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDAC7571
    float V_out = desidered_Voltage;
    float D_raw = (V_out * DAC_LEVELS) / V_DD;
    // Round the value to the nearest integer value.
    uint16_t D = (uint16_t)roundf(D_raw);
    // check if the number is in [0-4095] otherwise raise and error
    if (D < 0 || D > DAC_LEVELS - 1)
    {
        DEBUG_PRINT("Error: The value of the DAC is not in the range [0-4095]:%d\n", D);
        return 0;
    }

    return D;
}

// ----------------- Potentiometer function ------------------------------
uint16_t WiperResistanceValue_From_Interpolation(float Vdd)
{
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22147a.pdf

    float R_WB = ((5.5f - Vdd) / (5.5f - 2.7f)) * RW_2_7V + ((Vdd - 2.7f) / (5.5f - 2.7f)) * RW_5_5V;

    uint16_t D = (uint16_t)roundf(R_WB);
    return D;
}

float Potentiometer_Resistance_Value_from_Desidered_Gain(float desidered_Gain)
{
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22147a.pdf
    // this resistance is R12 in the schematic and it correspond to R_wb in formulas 6-2
    return (desidered_Gain - 1) * R10;
}

uint8_t Potentiometer_Value_To_Set(float desidered_Gain)
{
    // equation  6-2 datasheet  DS22147A
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/22147a.pdf

    float R_WB_from_Desidered_Gain = Potentiometer_Resistance_Value_from_Desidered_Gain(desidered_Gain);
    // DEBUG_PRINT("R_WB_from_Desidered_Gain: %f\n", R_WB_from_Desidered_Gain);
    // Step resistance (RS) is the resistance from one tap setting to the next. Values in  [4000-6000] Ohm, typical 5000 Ohm
    float R_S = POTENTIOMETER_FULL_SCALE_RAB / POTENTIOMETER_NUMBER_OF_STEPS;
    // Compute the Raw value of the N starting from equation 6-2
    uint16_t TYPICAL_DC_WIPER_RESISTANCE = WiperResistanceValue_From_Interpolation(V_DD);
    float N_raw = (R_WB_from_Desidered_Gain - TYPICAL_DC_WIPER_RESISTANCE) / R_S;
    // Round the value to the nearest integer value.
    uint8_t N = (uint8_t)roundf(N_raw);
    // check if the number is in [0-127] otherwise raise and error
    if (N > POTENTIOMETER_NUMBER_OF_STEPS)
    {
        DEBUG_PRINT("Error: The value of the potentiometer is not in the range [0-127]:%d\n", N);
        return 0;
    }
    return N;
}

// ---------------- DMA Interrupt Handler ------------------------------
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
        // DMA_Cmd(DMA_Stream, DISABLE);
        ADC_Cmd(ADC_n, DISABLE);
        // DEBUG_PRINT("ADC disabled\n");
        ADC_ContinuousModeCmd(ADC_n, DISABLE);
        DMA_ClearITPendingBit(DMA_Stream, DMA_IT_TCIF4);

        ADC_Done = 1;
    }
    if (DMA_GetITStatus(DMA_Stream, DMA_IT_TEIF4))
    {
        DEBUG_PRINT("DMA_IT_TEIF4\n");
        DMA_ClearITPendingBit(DMA_Stream, DMA_IT_TEIF4);
    }
}

void ADC1_IRQHandler(void)
{
    DEBUG_PRINT("ADC1_IRQHandler\n");
    if (ADC_GetFlagStatus(ADC_n, ADC_FLAG_OVR))
    {
        ADC_ClearFlag(ADC_n, ADC_FLAG_OVR);
    }
}

// --------------------------- Math Utils Functions ---------------------------
float dot_product(float *a, float *b, int length)
{
    float result = 0.0f;
    for (int i = 0; i < length; i++)
    {
        result += a[i] * b[i];
    }
    return result;
}

float euclidean_distance(float *a, float *b, int length)
{
    float sum = 0.0f;
    for (int i = 0; i < length; i++)
    {
        float diff = a[i] - b[i];
        sum += diff * diff;
    }
    return sqrtf(sum);
}

void getversor(float *a, float *b, float *u, int length)
{
    float d = euclidean_distance(a, b, length);
    for (int i = 0; i < length; i++)
    {
        u[i] = (a[i] - b[i]) / d;
    }
}

float computeSTD(float *data, int arrayDimension)
{
    float sum = 0.0;
    float mean = 0.0;
    float standardDeviation = 0.0;

    int i;

    for (i = 0; i < arrayDimension; i++)
    {
        sum += data[i];
    }

    mean = sum / (float)arrayDimension;

    for (i = 0; i < arrayDimension; i++)
        standardDeviation += powf(data[i] - mean, 2);

    return sqrtf(standardDeviation / (float)arrayDimension);
}

// function that given 3 points reonctruct a paraboloid and extract the peack
typedef struct
{
    float x;
    float y;
} Point_magnetic_fft;

void reconstructParabolaAndFindPeak(Point_magnetic_fft p1, Point_magnetic_fft p2, Point_magnetic_fft p3, Point_magnetic_fft *peak)
{
    // Matrice dei coefficienti
    float A[3][3] = {
        {p1.x * p1.x, p1.x, 1},
        {p2.x * p2.x, p2.x, 1},
        {p3.x * p3.x, p3.x, 1}};

    // Vettore dei termini noti
    float B[3] = {p1.y, p2.y, p3.y};

    // Risolvi il sistema di equazioni lineari A * [a, b, c]^T = B
    // Utilizza il metodo di eliminazione di Gauss

    // Per semplicità, assumiamo che la soluzione sia unica e non degenerata
    // Implementazione semplificata dell'eliminazione di Gauss
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            float ratio = A[j][i] / A[i][i];
            for (int k = 0; k < 3; k++)
            {
                A[j][k] -= ratio * A[i][k];
            }
            B[j] -= ratio * B[i];
        }
    }

    float c = B[2] / A[2][2];
    float b = (B[1] - A[1][2] * c) / A[1][1];
    float a = (B[0] - A[0][2] * c - A[0][1] * b) / A[0][0];

    // Il picco della parabola è nel vertice x = -b / (2a)
    peak->x = -b / (2 * a);
    peak->y = a * peak->x * peak->x + b * peak->x + c;
}
// ---------------- Measurement Model Functions ------------------------------
void get_B_field_for_a_Anchor(float *anchor_pos,
                              const float *tag_pos,
                              float *tag_or_versor,
                              float *B_field)
{

    // check on the tag pose if is it the origin, if so add a small value to the z coordinate
    float checkedTagPose[3];
    if (tag_pos[0] == 0.0f && tag_pos[1] == 0.0f && tag_pos[2] == 0.0f)
    {

        checkedTagPose[0] = tag_pos[0];
        checkedTagPose[1] = tag_pos[1];
        // printf("Tag position is the origin\n");
        checkedTagPose[2] = 0.0001f;
        // printf("TAG position is now %f %f %f\n", tag_pos[0], tag_pos[1], tag_pos[2]);
    }
    else
    {
        checkedTagPose[0] = tag_pos[0];
        checkedTagPose[1] = tag_pos[1];
        checkedTagPose[2] = tag_pos[2];
    }

    // printf("TAG orientation = %f %f %f\n", tag_or_versor[0], tag_or_versor[1], tag_or_versor[2]);
    // printf("N_WOUNDS %f\n", N_WOUNDS);
    // printf("COIL_SURFACE %f\n", COIL_SURFACE);
    // printf("CURRENT %f\n", CURRENT);

    float tx_rx_versor[3];
    getversor(anchor_pos, checkedTagPose, tx_rx_versor, 3);
    // printf("tx_rx_versor = %.8f\n", tx_rx_versor);

    float magnetic_dipole_moment_tx[3] = {
        N_WOUNDS * COIL_SURFACE * CURRENT * tag_or_versor[0],
        N_WOUNDS * COIL_SURFACE * CURRENT * tag_or_versor[1],
        N_WOUNDS * COIL_SURFACE * CURRENT * tag_or_versor[2]};
    // printf("magnetic_dipole_moment_tx = %.8f,%.8f,%.8f\n", magnetic_dipole_moment_tx[0], magnetic_dipole_moment_tx[1], magnetic_dipole_moment_tx[2]);

    float tx_rx_distance = euclidean_distance(anchor_pos, checkedTagPose, 3);

    // printf("tx_rx_distance = %.8f\n", tx_rx_distance);

    float dot_product_B_temp = dot_product(magnetic_dipole_moment_tx, tx_rx_versor, 3);
    // printf("dot_product_B_temp = %.8f\n", dot_product_B_temp);

    float constant_Bfield_constant_1 = (MU_0 / (4.0f * PI)) / powf(tx_rx_distance, 3);
    // printf("constant_Bfield_constant_1 = %.8f\n", constant_Bfield_constant_1);

    float B_temp[3] = {
        constant_Bfield_constant_1 * (3.0f * dot_product_B_temp * tx_rx_versor[0] - magnetic_dipole_moment_tx[0]),
        constant_Bfield_constant_1 * (3.0f * dot_product_B_temp * tx_rx_versor[1] - magnetic_dipole_moment_tx[1]),
        constant_Bfield_constant_1 * (3.0f * dot_product_B_temp * tx_rx_versor[2] - magnetic_dipole_moment_tx[2])};

    // float magnetic_dipole_moment_tx_magnitude = euclidean_distance(magnetic_dipole_moment_tx, magnetic_dipole_moment_tx, 3);
    // compute the norm of the magnetic dipole moment
    // float magnetic_dipole_moment_tx_magnitude = sqrtf(magnetic_dipole_moment_tx[0] * magnetic_dipole_moment_tx[0] +
    //                                                   magnetic_dipole_moment_tx[1] * magnetic_dipole_moment_tx[1] +
    //                                                   magnetic_dipole_moment_tx[2] * magnetic_dipole_moment_tx[2]);

    // normalizing the B field (NOT NEEDED!!!!)
    B_field[0] = B_temp[0]; // * magnetic_dipole_moment_tx_magnitude;
    B_field[1] = B_temp[1]; // * magnetic_dipole_moment_tx_magnitude;
    B_field[2] = B_temp[2]; // * magnetic_dipole_moment_tx_magnitude;
}

float V_from_B(float *B_field, float *rx_versor, float resonanceFreq, float Gain)
{
    float dot_product_V = dot_product(B_field, rx_versor, 3);
    // printf("dot_product_V = %.8f\n", dot_product_V);

    float b = fabsf(2.0f * PI * resonanceFreq * PI * RAY * RAY * N_WOUNDS * dot_product_V);
    // printf("b = %.8f\n", b);
    // printf("Gain = %.8f\n", Gain);

    float V = Gain * b;
    // printf("V = %.8f\n", V);
    return V;
}

// --------------------- Nelder-Mead functions ---------------------

const float myCostFunction_3A(int n, const float *x, void *arg)
{

    // cast the void pointer to what we expect to find
    myParams_t_3A *params = (myParams_t_3A *)arg;

    float costo = 0.0f;

    // compute the predicted voltage in in the initial point x

    // float startingx[3] = {x[0], x[1], x[2]};
    float B_field_vector_Optim[3];

    for (int anchorIdx = 0; anchorIdx < NUM_ANCHORS - 1; anchorIdx++)
    {

        get_B_field_for_a_Anchor(params->Anchors_3A[anchorIdx], x, params->versore_orientamento_cf_3A, B_field_vector_Optim);
        costo += powf(V_from_B(B_field_vector_Optim, params->versore_orientamento_cf_3A, params->frequencies_3A[anchorIdx], params->Gain_3A) - params->MeasuredVoltages_calibrated_3A[anchorIdx], 2);
    }

    return costo;
}
const float myCostFunction_4A(int n, const float *x, void *arg)
{

    // cast the void pointer to what we expect to find
    myParams_t_4A *params = (myParams_t_4A *)arg;

    float costo = 0.0f;

    // compute the predicted voltage in in the initial point x

    // float startingx[3] = {x[0], x[1], x[2]};
    float B_field_vector_Optim[3];

    for (int anchorIdx = 0; anchorIdx < NUM_ANCHORS; anchorIdx++)
    {

        get_B_field_for_a_Anchor(params->Anchors_4A[anchorIdx], x, params->versore_orientamento_cf_4A, B_field_vector_Optim);
        costo += powf(V_from_B(B_field_vector_Optim, params->versore_orientamento_cf_4A, params->frequencies_4A[anchorIdx], params->Gain_4A) - params->MeasuredVoltages_calibrated_4A[anchorIdx], 2);
    }

    return costo;
}
// ---------------------Grid Search ---------------------

// const float GridSearchCostFunction(int dx, int dy, float initial_x, float initial_y, float initial_z, float step, void *arg, float min_x, float min_y)
// {

//     // cast the void pointer to what we expect to find
//     myParams_t *myParams = (myParams_t *)arg;

//     // computing the value of the cost function in each point (of step step) of the give area (dx * dy) and the find the x and the y of the minimum value
//     float minCost = 1000000.0f;
//     float cost = 0.0f;
//     float x = initial_x;
//     float y = initial_y;
//     min_x = 1000000.0f;
//     min_y = 1000000.0f;

//     for (int i = 0; i < dx; i++)
//     {
//         for (int j = 0; j < dy; j++)
//         {
//             cost = myCostFunction(3, (const float[]){x, y, initial_z}, &myParams);
//             if (cost < minCost)
//             {
//                 minCost = cost;
//                 // Store the x and y values associated with the minimum cost
//                 min_x = x;
//                 min_y = y;
//             }
//             y += step;
//         }
//         x += step;
//         y = initial_y;
//     }
// }

// -------------------- Cycling Function ----------------------------

// Identify the saturation anchors
// Identify the saturation anchors
void check_saturations(FFT_Amplitudes *amplitudes, int *Id_in_saturation, bool *there_is_saturation)
{
    // ---------------------- SATURATION CASE -------------------------------------
    *there_is_saturation = false;
    Id_in_saturation[0] = 0;
    Id_in_saturation[1] = 0;
    Id_in_saturation[2] = 0;
    Id_in_saturation[3] = 0;

    if (amplitudes->Coil_1Ampl >= SATURATION_TRESHOLD)
    {
        Id_in_saturation[0] = 1;
        // DEBUG_PRINT("Saturating\n");
        *there_is_saturation = true;
    }
    if (amplitudes->Coil_2Ampl >= SATURATION_TRESHOLD)
    {
        Id_in_saturation[1] = 1;
        // DEBUG_PRINT("Saturating\n");
        *there_is_saturation = true;
    }
    if (amplitudes->Coil_3Ampl >= SATURATION_TRESHOLD)
    {
        Id_in_saturation[2] = 1;
        // DEBUG_PRINT("Saturating\n");
        *there_is_saturation = true;
    }
    if (amplitudes->Coil_4Ampl >= SATURATION_TRESHOLD)
    {
        Id_in_saturation[3] = 1;
        // DEBUG_PRINT("Saturating\n");
        *there_is_saturation = true;
    }
}

// fft and kalman update function
FFT_Amplitudes performFFT(uint32_t *Input_buffer_pointer, float32_t *Output_buffer_pointer, float32_t flattopCorrectionFactor)
{
    // if (skipThisMeasurement == 1)
    // {
    //     return;
    // }
    FFT_Amplitudes amps;

    arm_q31_to_float((q31_t *)Input_buffer_pointer, Output_buffer_pointer, FFT_SIZE);
    int p = 0;
    for (p = 0; p < FFT_SIZE; p++)
    {
        // Fattore di scala della conversione della funzione, guarda doc per capire
        Output_buffer_pointer[p] = Output_buffer_pointer[p] * 2147483648.0f;

        // ADCLevelsToVolt
        Output_buffer_pointer[p] = Output_buffer_pointer[p] * (ADC_MAX_VOLTAGE / ADC_LEVELS);
    }

    // fin the maximum of the Output_buffer_pointer
    // float maxSat;
    // uint32_t maxIndexSat;
    // arm_max_f32(Output_buffer_pointer + 1, FFT_SIZE - 1, &maxSat, &maxIndexSat);
    // maxIndexSat += 1;

    // apply flattop window
    arm_mult_f32(Output_buffer_pointer, (float32_t *)flattop_2048_lut, Output_buffer_pointer, FFT_SIZE);

    // perform FFT
    arm_rfft_fast_f32(&fft_instance, Output_buffer_pointer, fft_output, 0);

    // normalizing the fft output
    int i = 0;
    for (i = 0; i < FFT_SIZE; i++)
    {
        fft_output[i] = fft_output[i] / FFT_SIZE;
    }

    // computing the magnitude of the fft
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

    // Extract the max for each anchor
    // NOTE:*2 is because the fft of a sin is 2 impulsive delta of A/2 amplitude therefore to get the full amplitude of the signal i need to multiply by 2
    // NOTE: *flattopCorrectionFactor is the correction factor for the flattop window that i applied

    // Extract the maximum value and its index around Coil_1Idx
    uint32_t maxindex;
    arm_max_f32(&fft_magnitude[Coil_1Idx - 2], 4, &amps.Coil_1Ampl, &maxindex);
    // get the value before and the value after the maximum value
    float beforeCoil_1 = fft_magnitude[Coil_1Idx - 2];
    float afterCoil_1 = fft_magnitude[Coil_1Idx + 1];
    // define the points
    Point_magnetic_fft Coil_1Point = {Coil_1Idx, amps.Coil_1Ampl};
    Point_magnetic_fft beforeCoil_1Point = {Coil_1Idx - 2, beforeCoil_1};
    Point_magnetic_fft afterCoil_1Point = {Coil_1Idx + 1, afterCoil_1};
    // find the peach of the signal
    Point_magnetic_fft Coil_1MaxPoint;
    reconstructParabolaAndFindPeak(beforeCoil_1Point, Coil_1Point, afterCoil_1Point, &Coil_1MaxPoint);
    amps.Coil_1Ampl = Coil_1MaxPoint.y * flattopCorrectionFactor * 2.0f;

    // Calculate the maximum value and its index around Coil_2Idx
    arm_max_f32(&fft_magnitude[Coil_2Idx - 1], 3, &amps.Coil_2Ampl, &maxindex);
    // get the value before and the value after the maximum value
    float beforeCoil_2 = fft_magnitude[Coil_2Idx - 2];
    float afterCoil_2 = fft_magnitude[Coil_2Idx + 1];
    // define the points
    Point_magnetic_fft Coil_2Point = {Coil_2Idx, amps.Coil_2Ampl};
    Point_magnetic_fft beforeCoil_2Point = {Coil_2Idx - 2, beforeCoil_2};
    Point_magnetic_fft afterCoil_2Point = {Coil_2Idx + 1, afterCoil_2};
    // find the peach of the signal
    Point_magnetic_fft Coil_2MaxPoint;
    reconstructParabolaAndFindPeak(beforeCoil_2Point, Coil_2Point, afterCoil_2Point, &Coil_2MaxPoint);
    amps.Coil_2Ampl = Coil_2MaxPoint.y * flattopCorrectionFactor * 2.0f;

    // Calculate the maximum value and its index around Coil_3Idx
    arm_max_f32(&fft_magnitude[Coil_3Idx - 1], 3, &amps.Coil_3Ampl, &maxindex);
    // DEBUG_PRINT("MaxIndex: %d\n",maxindex);
    // DEBUG_PRINT("amps.Coil_3Ampl: %f\n",amps.Coil_3Ampl);
    float beforeCoil_3 = fft_magnitude[Coil_3Idx - 2];
    float afterCoil_3 = fft_magnitude[Coil_3Idx + 1];
    // define the points
    Point_magnetic_fft Coil_3Point = {Coil_3Idx, amps.Coil_3Ampl};
    Point_magnetic_fft beforeCoil_3Point = {Coil_3Idx - 2, beforeCoil_3};
    Point_magnetic_fft afterCoil_3Point = {Coil_3Idx + 1, afterCoil_3};
    // find the peach of the signal
    Point_magnetic_fft Coil_3MaxPoint;
    reconstructParabolaAndFindPeak(beforeCoil_3Point, Coil_3Point, afterCoil_3Point, &Coil_3MaxPoint);
    // DEBUG_PRINT("Coil_3MaxPoint.y: %f\n",Coil_3MaxPoint.y);
    amps.Coil_3Ampl = Coil_3MaxPoint.y * flattopCorrectionFactor * 2.0f;

    // Calculate the maximum value and its index around Coil_4Idx
    arm_max_f32(&fft_magnitude[Coil_4Idx - 1], 3, &amps.Coil_4Ampl, &maxindex);
    float beforeCoil_4 = fft_magnitude[Coil_4Idx - 2];
    float afterCoil_4 = fft_magnitude[Coil_4Idx + 1];
    // define the points
    Point_magnetic_fft Coil_4Point = {Coil_4Idx, amps.Coil_4Ampl};
    Point_magnetic_fft beforeCoil_4Point = {Coil_4Idx - 2, beforeCoil_4};
    Point_magnetic_fft afterCoil_4Point = {Coil_4Idx + 1, afterCoil_4};
    // find the peach of the signal
    Point_magnetic_fft Coil_4MaxPoint;
    reconstructParabolaAndFindPeak(beforeCoil_4Point, Coil_4Point, afterCoil_4Point, &Coil_4MaxPoint);
    amps.Coil_4Ampl = Coil_4MaxPoint.y * flattopCorrectionFactor * 2.0f;

    amps.AllAmpl[0] = amps.Coil_1Ampl;
    amps.AllAmpl[1] = amps.Coil_2Ampl;
    amps.AllAmpl[2] = amps.Coil_3Ampl;
    amps.AllAmpl[3] = amps.Coil_4Ampl;

    // return the amplitudes
    return amps;

    // --------------------------------- 2D MEASUREMENT MODEL - DISTANCE COMPUTATION ---------------------------------
    /*

    // THIS MODEL IS NOT USED TO UPDATE THE KALMAN FILTER, JUST AS CONFIRMATION

    // compute the the distances from the amplitude of each anchor
    Coil_1_distance = powf(10, (log10(Coil_1Ampl) - Coil_1_Q) / Coil_1_M);
    Coil_2_distance = powf(10, (log10(Coil_2Ampl) - Coil_2_Q) / Coil_2_M);
    Coil_3_distance = powf(10, (log10(Coil_3Ampl) - Coil_3_Q) / Coil_3_M);
    Coil_4_distance = powf(10, (log10(Coil_4Ampl) - Coil_4_Q) / Coil_4_M);

    // Coil_1
    distanceMeasurement_t dist_Coil_1;
    dist_Coil_1.distance = Coil_1_distance;
    dist_Coil_1.x = Coil_1_Position[0];
    dist_Coil_1.y = Coil_1_Position[1];
    dist_Coil_1.z = Coil_1_Position[2];
    dist_Coil_1.anchorId = Coil_1_Id;
    dist_Coil_1.stdDev = MagneticStandardDeviation;
    // DEBUG_PRINT("Coil_1 Distance: %f\n", Coil_1_distance);
    // estimatorEnqueueDistance(&dist_Coil_1);

    // // Coil_2
    distanceMeasurement_t dist_Coil_2;
    dist_Coil_2.distance = Coil_2_distance;
    dist_Coil_2.x = Coil_2_Position[0];
    dist_Coil_2.y = Coil_2_Position[1];
    dist_Coil_2.z = Coil_2_Position[2];
    dist_Coil_2.anchorId = Coil_2_Id;
    dist_Coil_2.stdDev = MagneticStandardDeviation;
    // estimatorEnqueueDistance(&dist_Coil_2);

    // // Coil_3
    distanceMeasurement_t dist_Coil_3;
    dist_Coil_3.distance = Coil_3_distance;
    dist_Coil_3.x = Coil_3_Position[0];
    dist_Coil_3.y = Coil_3_Position[1];
    dist_Coil_3.z = Coil_3_Position[2];
    dist_Coil_3.anchorId = Coil_3_Id;
    dist_Coil_3.stdDev = MagneticStandardDeviation;
    // estimatorEnqueueDistance(&dist_Coil_3);

    // // Coil_4
    distanceMeasurement_t dist_Coil_4;
    dist_Coil_4.distance = Coil_4_distance;
    dist_Coil_4.x = Coil_4_Position[0];
    dist_Coil_4.y = Coil_4_Position[1];
    dist_Coil_4.z = Coil_4_Position[2];
    dist_Coil_4.anchorId = Coil_4_Id;
    dist_Coil_4.stdDev = MagneticStandardDeviation;
    estimatorEnqueueDistance(&dist_Coil_4);
    */

    // ---------------3D MEASUREMENT MODEL - POSITION COMPUTATION------------------------------
    // THIS MODEL IS USED TO UPDATE THE KALMAN FILTER

    // voltMeasurement_t volt;

    // volt.x[0] = Coil_1_Position_x;
    // volt.y[0] = Coil_1_Position_y;
    // volt.z[0] = Coil_1_Position_z;
    // volt.stdDev[0] = MagneticStandardDeviation;
    // volt.measuredVolt[0] = Coil_1Ampl;
    // volt.anchorId[0] = Coil_1_Id;
    // volt.resonanceFrequency[0] = Coil_1ResFreq;
    // volt.GainValue = TotalGain;

    // volt.x[1] = Coil_2_Position_x;
    // volt.y[1] = Coil_2_Position_y;
    // volt.z[1] = Coil_2_Position_z;
    // volt.stdDev[1] = MagneticStandardDeviation;
    // volt.measuredVolt[1] = Coil_2Ampl;
    // volt.anchorId[1] = Coil_2_Id;
    // volt.resonanceFrequency[1] = Coil_2ResFreq;
    // volt.GainValue = TotalGain;

    // volt.x[2] = Coil_3_Position_x;
    // volt.y[2] = Coil_3_Position_y;
    // volt.z[2] = Coil_3_Position_z;
    // volt.stdDev[2] = MagneticStandardDeviation;
    // volt.measuredVolt[2] = Coil_3Ampl;
    // volt.anchorId[2] = Coil_3_Id;
    // volt.resonanceFrequency[2] = Coil_3ResFreq;
    // volt.GainValue = TotalGain;

    // volt.x[3] = Coil_4_Position_x;
    // volt.y[3] = Coil_4_Position_y;
    // volt.z[3] = Coil_4_Position_z;
    // volt.stdDev[3] = MagneticStandardDeviation;
    // volt.measuredVolt[3] = Coil_4Ampl;
    // volt.anchorId[3] = Coil_4_Id;
    // volt.resonanceFrequency[3] = Coil_4ResFreq;
    // volt.GainValue = TotalGain;

    // check if the ADC is saturating
    // TODO: 2.4 Voltages max voltages before saturation, so if you see 2.4V in the output, then change the gain. Also viceversa, to be tested the mimumum value

    // ---------------------- SATURATION CASE -------------------------------------

    // volt.there_is_saturation = false;
    // volt.Id_in_saturation[0] = 0;
    // volt.Id_in_saturation[1] = 0;
    // volt.Id_in_saturation[2] = 0;
    // volt.Id_in_saturation[3] = 0;

    // if (Coil_1Ampl >= SATURATION_TRESHOLD)
    // {
    //     volt.Id_in_saturation[0] = 1;
    //     // DEBUG_PRINT("Saturating\n");
    //     volt.there_is_saturation = true;
    // }
    // if (Coil_2Ampl >= SATURATION_TRESHOLD)
    // {
    //     volt.Id_in_saturation[1] = 1;
    //     // DEBUG_PRINT("Saturating\n");
    //     volt.there_is_saturation = true;
    // }
    // if (Coil_3Ampl >= SATURATION_TRESHOLD)
    // {
    //     volt.Id_in_saturation[2] = 1;
    //     // DEBUG_PRINT("Saturating\n");
    //     volt.there_is_saturation = true;
    // }
    // if (Coil_4Ampl >= SATURATION_TRESHOLD)
    // {
    //     volt.Id_in_saturation[3] = 1;
    //     // DEBUG_PRINT("Saturating\n");
    //     volt.there_is_saturation = true;
    // }

    // // check if the max is saturating
    // if (maxSat >= 1.2f)
    // {
    //     // if the max is saturating, then skip this measurement

    //     // if the max is saturating, then skip this measurement
    //     if (counterSaturation % 10 == 0)
    //     {
    //         DEBUG_PRINT("Saturating\n");
    //         // DEBUG_PRINT("Max Value: %f\n", (double)maxSat);
    //         // DEBUG_PRINT("Max Index: %d\n", (int)maxIndexSat);
    //     }
    //     counterSaturation++;

    //     // finde the ancor more close to the max by looking at the max fft value
    //     // Find the maximum value among Coil_1Ampl, Coil_2Ampl, Coil_3Ampl, Coil_4Ampl

    //     float maxAmpl = Coil_1Ampl;
    //     int maxAnchorId = Coil_1_Id;
    //     if (Coil_2Ampl > maxAmpl)
    //     {
    //         maxAmpl = Coil_2Ampl;
    //         maxAnchorId = Coil_2_Id;
    //     }
    //     if (Coil_3Ampl > maxAmpl)
    //     {
    //         maxAmpl = Coil_3Ampl;
    //         maxAnchorId = Coil_3_Id;
    //     }
    //     if (Coil_4Ampl > maxAmpl)
    //     {
    //         maxAmpl = Coil_4Ampl;
    //         maxAnchorId = Coil_4_Id;
    //     }

    //     char *anchorName;
    //     switch (maxAnchorId)
    //     {
    //     case Coil_1_Id:
    //         anchorName = "Coil_1";
    //         break;
    //     case Coil_2_Id:
    //         anchorName = "Coil_2";
    //         break;
    //     case Coil_3_Id:
    //         anchorName = "Coil_3";
    //         break;
    //     case Coil_4_Id:
    //         anchorName = "Coil_4";
    //         break;
    //     default:
    //         anchorName = "Unknown";
    //         break;
    //     }
    //     // DEBUG_PRINT("Max Anchor Name: %s\n", anchorName);
    //     // Incresing the std for that measurement

    //     // volt.stdDev[maxAnchorId] = MagneticStandardDeviation * 2;

    //     volt.Id_in_saturation = maxAnchorId;
    //     isSaturated = 1;

    //     // DEBUG_PRINT("STD increased for %s *2\n", anchorName);
    // }
    // estimatorEnqueueVolt(&volt);

    // if (isSaturated == 0)
    // {
    //     estimatorEnqueueVolt(&volt);
    //     // float b = 0;
    // }
}

// finalization of main cycle
void finalizeCycle()
{
    DMA_Cmd(DMA_Stream, ENABLE);
    ADC_DMA_start(ADC_n, ADC_Channel, 1, ADC_SampleTime_15Cycles);
    skipThisMeasurement = 0;
    ADC_Done = 0;
}

static void mytask(void *param)
{
    DEBUG_PRINT("Wait for system starting\n");
    systemWaitStart();
    DEBUG_PRINT("System Started\n");

    // DAC Setup
    // SETTING the reference voltage for the DAC using i2c
    uint16_t ValueforDAC = ValueforDAC_from_DesideredVolt(V_REF_CRAZYFLIE / 2);
    uint8_t DAC_Write[2] = {0, 0};
    DAC_Write[1] = ValueforDAC & 0x00FF;
    DAC_Write[0] = (ValueforDAC >> 8) & 0x00FF;
    DEBUG_PRINT("Value for DAC: %d\n", ValueforDAC);
    uint8_t dacWriteResult = i2cdevWrite(I2C1_DEV, DECK_DAC_I2C_ADDRESS, DAC_WRITE_LENGTH, &DAC_Write[0]);
    // wait some time that the Hw is setted
    vTaskDelay(M2T(DAC_ANALOG_HW_DELAY_AFTER_SET));
    DEBUG_PRINT("DAC Write result: %d\n", dacWriteResult);

    // Potentiometer Setup
    uint8_t Potentiometer_Value = Potentiometer_Value_To_Set(GainValue);
    DEBUG_PRINT("Potentiometer Value to set: %d\n", Potentiometer_Value);
    uint8_t result_read_potentiometer = i2cdevWrite(I2C1_DEV, POTENTIOMETER_ADDR, 1, &Potentiometer_Value);
    vTaskDelay(M2T(POTENTIOMETER_ANALOG_HW_DELAY_AFTER_SET));
    DEBUG_PRINT("Potentiometer Write result: %d\n", result_read_potentiometer);
    GainValue_Setted = GainValue;
    GainValue = 0;
    TotalGain = GainValue_Setted * OpAmpGainValue;

    // reading the value of the potentiometer
    uint8_t PotentiometerReadValue = 10;
    uint8_t result_read = i2cdevRead(I2C1_DEV, POTENTIOMETER_ADDR, 1, &PotentiometerReadValue);

    vTaskDelay(M2T(POTENTIOMETER_ANALOG_HW_DELAY_AFTER_SET));
    DEBUG_PRINT("Potentiometer Readed result: %d\n", result_read);
    DEBUG_PRINT("Potentiometer Read Value: %d\n", PotentiometerReadValue);

    // gpio init
    GPIO_init_analog(DECK_GPIO_RX2);
    // DMA init
    DMA_inititalization(RCC_AHB1Periph_DMA2, DMA_Stream, DMA_Buffer, ADC_n, DMA_Channel, DMA_IRQ, ARRAY_SIZE);
    // adc init
    ADC_init_DMA_mode(RCC_APB2Periph_ADC1, ADC_n);
    // Call the ADC_DMA_start function
    ADC_DMA_start(ADC_n, ADC_Channel, 1, ADC_SampleTime_15Cycles);
    // Initialize the FFT instance
    arm_rfft_fast_init_f32(&fft_instance, fft_length);

    // Flattop correction factor calculation
    float32_t sum = 0.0;
    for (int i = 0; i < ARRAY_SIZE; i++)
    {
        sum += flattop_2048_lut[i];
    }
    float32_t flattopCorrectionFactor = ARRAY_SIZE / sum;
    DEBUG_PRINT("Flattop Correction Factor: %f\n", (double)flattopCorrectionFactor);

    // ------------------ initialize the NM model params ------------------
    srand(time(NULL));

    static nm_params_t_3A paramsNM_3A;
    static nm_params_t_4A paramsNM_4A;

    nm_params_init_default_3A(&paramsNM_3A, 3);
    paramsNM_3A.debug_log = 0;
    paramsNM_3A.max_iterations = 20;
    paramsNM_3A.tol_fx = 1e-6f;
    paramsNM_3A.tol_x = 1e-5f;
    paramsNM_3A.restarts = 0;

    nm_params_init_default_4A(&paramsNM_4A, 3);
    paramsNM_4A.debug_log = 0;
    paramsNM_4A.max_iterations = 20;
    paramsNM_4A.tol_fx = 1e-6f;
    paramsNM_4A.tol_x = 1e-5f;
    paramsNM_4A.restarts = 0;

    bool isFirstMeasurement = true;

    // ----------------- Outlier -----------------
    static float euclidean_distance_xy = 0.0f;
    // static float euclidean_distance_z = 0.0f;

    // -----------------------  initialize the calibration params -----------------------

    float calibrationMean[4] = {};
    float calibrationsGains[4];

    // ----------------------- initialize the B field vectors -----------------------
    static float B_field_vector_1[3];
    static float B_field_vector_2[3];
    static float B_field_vector_3[3];
    static float B_field_vector_4[3];

    // ----------------------- initialize the Linear kalman filter -----------------------

    // initialize the kalman filter in 0,0,0

    float32_t initial_state_kalman_filter[3] = {0.0f, 0.0f, 0.0f};

    kalman_init(&kf, initial_state_kalman_filter);

    DEBUG_PRINT("Bin_size: %d\n", (int)BIN_SIZE);

    // ----------------------- moving average z-measurements -----------------------
    int numberOfZMeasurements = 0;
    float ZMeasurements[10];

    while (1)
    {
        // uint64_t start_cost = usecTimestamp();

        if (GainValue > 0)
        {
            DEBUG_PRINT("UPDATE FROM USER ON GAIN!!!!\n");
            // Potentiometer Setup
            uint8_t Potentiometer_Value = Potentiometer_Value_To_Set(GainValue);
            DEBUG_PRINT("Potentiometer Value to set: %d\n", Potentiometer_Value);
            uint8_t result_read_potentiometer = i2cdevWrite(I2C1_DEV, POTENTIOMETER_ADDR, 1, &Potentiometer_Value);
            vTaskDelay(M2T(POTENTIOMETER_ANALOG_HW_DELAY_AFTER_SET));
            DEBUG_PRINT("Potentiometer Write result: %d\n", result_read_potentiometer);
            GainValue_Setted = GainValue;
            GainValue = 0;
            TotalGain = GainValue_Setted * OpAmpGainValue;
        }
        if (ADC_Done == 1 && GainValue == 0)
        {
            if (skipThisMeasurement == 0)
            {
                // define the struct for the data to be processed
                // voltMeasurement_t volt;

                int Id_in_saturation[4];
                bool there_is_saturation = false;

                // The DMA buffer is full, perform the FFT
                FFT_Amplitudes amplitudes = performFFT(DMA_Buffer, fft_input, flattopCorrectionFactor);

                // check if the ADC is saturating and handle it
                check_saturations(&amplitudes, Id_in_saturation, &there_is_saturation);

                // Calibration

                if (currentCalibrationTick < CALIBRATION_TIC_VALUE)
                {
                    calibrationMean[0] += amplitudes.Coil_1Ampl;
                    calibrationMean[1] += amplitudes.Coil_2Ampl;
                    calibrationMean[2] += amplitudes.Coil_3Ampl;
                    calibrationMean[3] += amplitudes.Coil_4Ampl;

                    currentCalibrationTick = currentCalibrationTick + 1;
                }
                else
                {
                    if (currentCalibrationTick == CALIBRATION_TIC_VALUE)
                    {
                        // ------------------------------ CALIBRATION ------------------------------
                        // For  calibration i fixed the tag position and orientation to 0,0,0 and 0,0,1
                        currentCalibrationTick = currentCalibrationTick + 1;

                        float meanData_a1 = calibrationMean[0] / CALIBRATION_TIC_VALUE;
                        float meanData_a2 = calibrationMean[1] / CALIBRATION_TIC_VALUE;
                        float meanData_a3 = calibrationMean[2] / CALIBRATION_TIC_VALUE;
                        float meanData_a4 = calibrationMean[3] / CALIBRATION_TIC_VALUE;

                        // DEBUG_PRINT("calibration data acquired!!\n");
                        DEBUG_PRINT("calibration_Mean[0][0] = %f\n", (double)meanData_a1);
                        DEBUG_PRINT("calibration_Mean[1][1] = %f\n", (double)meanData_a2);
                        DEBUG_PRINT("calibration_Mean[2][2] = %f\n", (double)meanData_a3);
                        DEBUG_PRINT("calibration_Mean[3][3] = %f\n", (double)meanData_a4);

                        // point_t cfPosP;
                        // estimatorKalmanGetEstimatedPos(&cfPosP);
                        // float tag_pos_predicted_calibrated[3] = {cfPosP.x, cfPosP.y, cfPosP.z};
                        float tag_pos_predicted_calibrated[3] = {0.0f, 0.0f, 0.01f + offsetCoil};

                        // float RotationMatrix[3][3];
                        // estimatorKalmanGetEstimatedRot((float *)RotationMatrix);
                        // float tag_or_versor_calibrated[3] = {RotationMatrix[0][2], RotationMatrix[1][2], RotationMatrix[2][2]};
                        float tag_or_versor_calibrated[3] = {0.0f, 0.0f, 1.0f};

                        float anchor_1_pose[3] = {Coil_1_Position_x, Coil_1_Position_y, Coil_1_Position_z};
                        float anchor_2_pose[3] = {Coil_2_Position_x, Coil_2_Position_y, Coil_2_Position_z};
                        float anchor_3_pose[3] = {Coil_3_Position_x, Coil_3_Position_y, Coil_3_Position_z};
                        float anchor_4_pose[3] = {Coil_4_Position_x, Coil_4_Position_y, Coil_4_Position_z};
                        get_B_field_for_a_Anchor(anchor_1_pose, tag_pos_predicted_calibrated, tag_or_versor_calibrated, B_field_vector_1);
                        get_B_field_for_a_Anchor(anchor_2_pose, tag_pos_predicted_calibrated, tag_or_versor_calibrated, B_field_vector_2);
                        get_B_field_for_a_Anchor(anchor_3_pose, tag_pos_predicted_calibrated, tag_or_versor_calibrated, B_field_vector_3);
                        get_B_field_for_a_Anchor(anchor_4_pose, tag_pos_predicted_calibrated, tag_or_versor_calibrated, B_field_vector_4);

                        // computing the V_rx for each of the 4 anchors
                        float V_rx_1 = V_from_B(B_field_vector_1, tag_or_versor_calibrated, Coil_1ResFreq, TotalGain);
                        float V_rx_2 = V_from_B(B_field_vector_2, tag_or_versor_calibrated, Coil_2ResFreq, TotalGain);
                        float V_rx_3 = V_from_B(B_field_vector_3, tag_or_versor_calibrated, Coil_3ResFreq, TotalGain);
                        float V_rx_4 = V_from_B(B_field_vector_4, tag_or_versor_calibrated, Coil_4ResFreq, TotalGain);

                        calibrationsGains[0] = meanData_a1 / V_rx_1;
                        calibrationsGains[1] = meanData_a2 / V_rx_2;
                        calibrationsGains[2] = meanData_a3 / V_rx_3;
                        calibrationsGains[3] = meanData_a4 / V_rx_4;

                        DEBUG_PRINT("CG_a1 = %f\n", (double)calibrationsGains[0]);
                        DEBUG_PRINT("CG_a2 = %f\n", (double)calibrationsGains[1]);
                        DEBUG_PRINT("CG_a3 = %f\n", (double)calibrationsGains[2]);
                        DEBUG_PRINT("CG_a4 = %f\n", (double)calibrationsGains[3]);
                    }
                    else
                    {

                        if (currentCalibrationTick == CALIBRATION_TIC_VALUE + 1)
                        {
                            DEBUG_PRINT("currentCalibrationTick = %f\n", (double)currentCalibrationTick);

                            paramSetInt(paramGetVarId("kalman", "resetEstimation"), 1);
                            vTaskDelay(M2T(10));
                            currentCalibrationTick = currentCalibrationTick + 1;
                            DEBUG_PRINT("Resetting the Kalman filter after calibrationr\n");
                            paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
                            vTaskDelay(M2T(10));

                            // reset the mean accumulator variables
                            calibrationMean[0] = 0.0f;
                            calibrationMean[1] = 0.0f;
                            calibrationMean[2] = 0.0f;
                            calibrationMean[3] = 0.0f;

                            // paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
                        }

                        // ------------------------------------ RUNNING CASE ------------------------------------

                        float RotationMatrix[3][3];
                        // it would be the product between the rotation matrix and the initial [0,0,1] versor
                        estimatorKalmanGetEstimatedRot((float *)RotationMatrix);
                        float tag_or_versor[3] = {RotationMatrix[0][2], RotationMatrix[1][2], RotationMatrix[2][2]};

                        /// ------------------------- optimization  for computing position -------------------------

                        // initialize the params for the optimization algorithm
                        float x_start[3] = {};

                        if (isFirstMeasurement)
                        {
                            DEBUG_PRINT("First measurement\n");
                            x_start[0] = 0.0f;
                            x_start[1] = 0.0f;
                            x_start[2] = 0.01f + offsetCoil;
                            isFirstMeasurement = false;
                        }
                        else
                        {
                            if (initializationOnOrigin)
                            {
                                x_start[0] = 0.0f;
                                x_start[1] = 0.0f;
                                x_start[2] = 0.01f + offsetCoil;
                            }
                            else
                            {

                                point_t cfPosP;
                                estimatorKalmanGetEstimatedPos(&cfPosP);
                                x_start[0] = cfPosP.x;
                                x_start[1] = cfPosP.y;
                                x_start[2] = cfPosP.z + offsetCoil;
                            }
                        }

                        if (there_is_saturation)
                        {

                            myParams_t_3A my_params;
                            for (int anchorIdx = 0; anchorIdx < NUM_ANCHORS - 1; anchorIdx++)
                            {
                                // check if the anchor is not in saturation
                                // NOTE: ASSUMING MAX 1 ANCHOR IN SATURATION AT A TIME
                                if (Id_in_saturation[anchorIdx] == 0)
                                {
                                    my_params.Anchors_3A[anchorIdx][0] = AnchorPositionMatrix[anchorIdx][0];
                                    my_params.Anchors_3A[anchorIdx][1] = AnchorPositionMatrix[anchorIdx][1];
                                    my_params.Anchors_3A[anchorIdx][2] = AnchorPositionMatrix[anchorIdx][2];

                                    my_params.versore_orientamento_cf_3A[0] = tag_or_versor[0];
                                    my_params.versore_orientamento_cf_3A[1] = tag_or_versor[1];
                                    my_params.versore_orientamento_cf_3A[2] = tag_or_versor[2];

                                    my_params.Gain_3A = TotalGain;

                                    my_params.frequencies_3A[anchorIdx] = ResonanceFreqs[anchorIdx];
                                    // using the measured volts to fill the structure
                                    my_params.MeasuredVoltages_calibrated_3A[anchorIdx] = amplitudes.AllAmpl[anchorIdx] / calibrationsGains[anchorIdx];

                                    // debug
                                    MeasuredVoltages_calibrated[anchorIdx] = my_params.MeasuredVoltages_calibrated_3A[anchorIdx];
                                }
                            }
                            // float my_params_ms = (float)(usecTimestamp() - my_params_start_cost_all) / 1000.0f;

                            // set the starting point for the optimization algorithm

                            // float optimize_start_cost_all = usecTimestamp();
                            nm_multivar_optimize_3A(3, x_start, range, &myCostFunction_3A, &my_params, &paramsNM_3A, solution);
                            // float optimize_ms = (float)(usecTimestamp() - optimize_start_cost_all) / 1000.0f;
                        }
                        else
                        {
                            myParams_t_4A my_params;
                            for (int anchorIdx = 0; anchorIdx < NUM_ANCHORS; anchorIdx++)
                            {
                                my_params.Anchors_4A[anchorIdx][0] = AnchorPositionMatrix[anchorIdx][0];
                                my_params.Anchors_4A[anchorIdx][1] = AnchorPositionMatrix[anchorIdx][1];
                                my_params.Anchors_4A[anchorIdx][2] = AnchorPositionMatrix[anchorIdx][2];

                                my_params.versore_orientamento_cf_4A[0] = tag_or_versor[0];
                                my_params.versore_orientamento_cf_4A[1] = tag_or_versor[1];
                                my_params.versore_orientamento_cf_4A[2] = tag_or_versor[2];

                                my_params.Gain_4A = TotalGain;

                                my_params.frequencies_4A[anchorIdx] = ResonanceFreqs[anchorIdx];
                                // using the measured volts to fill the structure
                                my_params.MeasuredVoltages_calibrated_4A[anchorIdx] = amplitudes.AllAmpl[anchorIdx] / calibrationsGains[anchorIdx];
                                // debug
                                MeasuredVoltages_calibrated[anchorIdx] = my_params.MeasuredVoltages_calibrated_4A[anchorIdx];
                            }
                            // float my_params_ms = (float)(usecTimestamp() - my_params_start_cost_all) / 1000.0f;

                            // set the starting point for the optimization algorithm

                            // float optimize_start_cost_all = usecTimestamp();
                            nm_multivar_optimize_4A(3, x_start, range, &myCostFunction_4A, &my_params, &paramsNM_4A, solution);
                            // float optimize_ms = (float)(usecTimestamp() - optimize_start_cost_all) / 1000.0f;
                        }

                        static positionMeasurement_t ext_pos;

                        if (MODEL_TO_USE == 0)
                        /// ------------------------- optimization  for computing position -------------------------
                        {

                            // Outlier Detection, computing the euclidean distance between the estimated position by EKF and the solution
                            // xy
                            euclidean_distance_xy = euclidean_distance(x_start, solution, 2);

                            // check outlier on z
                            float zTreshold = 0.1f;
                            if (solution[2] - x_start[2] > zTreshold || solution[2] - x_start[2] < -zTreshold)
                            {
                                Optimization_Model_STD_Z = 0.3f;
                            }
                            else
                            {
                                // is not an outlier

                                //  now check if the drone is taking off or is landing in both cases the z is not reliable
                                // point_t cfPosP;
                                // estimatorKalmanGetEstimatedPos(&cfPosP);
                                if (x_start[2] < TARGET_FLYING_HEIGHT - 0.08f)
                                {
                                    Optimization_Model_STD_Z = 0.08f;
                                }
                                else
                                {
                                    Optimization_Model_STD_Z = 0.08f;
                                }
                            }

                            if (!(euclidean_distance_xy >= 0.50f))
                            {

                                // z_final_measurement = z_final_measurement - 0.1f; // maunual calibration z-azis

                                euclidean_distance_xy = 0.0f;

                                ext_pos.x = solution[0];
                                ext_pos.y = solution[1];
                                // ext_pos.z = z_final_measurement;
                                ext_pos.z = solution[2]; // - offsetCoil; // NOTA: è COMPENSATO PRIMA ORA
                                ext_pos.stdDev = Optimization_Model_STD;
                                estimatorEnqueuePosition(&ext_pos);
                                outlier = 0;
                            }
                            else
                            {
                                outlier = 1;
                            }
                        }
                        else if (MODEL_TO_USE == 1)
                        /// ------------------------- linear kalman filter --------------------------------
                        {
                            // vTaskDelay(M2T(1));
                            // kalman_update_measurements(solution);
                            // kalman_update();
                            // kalman_predict();
                            // ext_pos.x = solution[0];
                            // ext_pos.y = solution[1];
                            // ext_pos.z = solution[2];
                            // ext_pos.stdDev = Optimization_Model_STD;

                            kf.z_meas[0] = solution[0];
                            kf.z_meas[1] = solution[1];
                            kf.z_meas[2] = solution[2];

                            kalman_predict(&kf);
                            kalman_update(&kf);

                            // DEBUG_PRINT("KF x_state[0] = %f\n", (double)kf.x_state[0]);
                            // DEBUG_PRINT("KF x_state[1] = %f\n", (double)kf.x_state[1]);
                            // DEBUG_PRINT("KF x_state[2] = %f\n", (double)kf.x_state[2]);

                            ext_pos.x = kf.x_state[0];
                            ext_pos.y = kf.x_state[1];
                            ext_pos.z = kf.x_state[2];

                            LKF_ESTIMATION_DEBUG[0] = ext_pos.x;
                            LKF_ESTIMATION_DEBUG[1] = ext_pos.y;
                            LKF_ESTIMATION_DEBUG[2] = ext_pos.z;

                            // DEBUG_PRINT("LKF_ESTIMATION_DEBUG[0] = %f\n", (double)LKF_ESTIMATION_DEBUG[0]);
                            // DEBUG_PRINT("LKF_ESTIMATION_DEBUG[1] = %f\n", (double)LKF_ESTIMATION_DEBUG[1]);
                            // DEBUG_PRINT("LKF_ESTIMATION_DEBUG[2] = %f\n", (double)LKF_ESTIMATION_DEBUG[2]);

                            ext_pos.stdDev = Optimization_Model_STD;

                            estimatorEnqueuePosition(&ext_pos);
                        }
                    }
                }
            }
            // Iteration is done, restart the ADC, reset variables
            finalizeCycle();
        }
        else
        {
            // SOMETIME THE ADC IS BLOCKED, SO I NEED TO RESTART IT MANUALLY, SEEMS NOT APPENING ANYMORE
            // ------------DEBUGGING STUFF--------------

            // NON È NESSUNA DI QUESTE CONDIZIONI

            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)
            // {
            //     // Overrun occurred
            //     // Take appropriate actions to handle the overrun
            //     DEBUG_PRINT("Overrun occurred\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_OVR); // Clear the overrun flag
            //     ADC_Done = 1;
            // }
            //             ADC_ClearFlag(ADC_n, ADC_FLAG_AWD);
            // ADC_ClearFlag(ADC_n, ADC_FLAG_EOC);
            // ADC_ClearFlag(ADC_n, ADC_FLAG_JEOC);
            // ADC_ClearFlag(ADC_n, ADC_FLAG_JSTRT);
            // ADC_ClearFlag(ADC_n, ADC_FLAG_STRT);
            // ADC_ClearFlag(ADC_n, ADC_FLAG_OVR);
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_STRT) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_STRT\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_STRT);
            //     ADC_Done = 1;
            // }
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_EOC\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
            //     ADC_Done = 1;
            // }
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_JEOC\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
            //     ADC_Done = 1;
            // }
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JSTRT) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_JSTRT\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_JSTRT);
            //     ADC_Done = 1;
            // }
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_AWD) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_AWD\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
            //     ADC_Done = 1;
            // }
            // if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)
            // {
            //     DEBUG_PRINT("ADC_FLAG_OVR\n");
            //     ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
            //     ADC_Done = 1;
            // }

            // Se si blocca l'adc si puo' usare questo
            // non ha senso!!!
            ADC_Done = 1;
            // DEBUG_PRINT("ADC_Done: %d\n", ADC_Done);
            skipThisMeasurement = 1;
            // ma funziona cosi, non so poi come siano i dati
        }

        // Defining the delay between the executions
        // float diff_in_ms = (float)(usecTimestamp() - start_cost) / 1000.0f;

        // // print every 10 cycles
        // if (counter % 10 == 0)
        // {
        //     DEBUG_PRINT("Time for the cycle in ms: %f\n", (double)diff_in_ms);
        // }

        vTaskDelay(M2T(SYSTEM_PERIOD_MS));
        counter++;
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

    isInit = true;
}

static bool magneticTest()
{
    DEBUG_PRINT("MAGNETIC test passed!\n");
    return true;
}

static const DeckDriver magneticDriver = {
    .name = "MagneticDeck",
    .usedGpio = DECK_USING_PA3,
    .usedPeriph = DECK_USING_I2C,
    .init = magneticInit,
    .test = magneticTest,
};

DECK_DRIVER(magneticDriver);

#define CONFIG_DEBUG_LOG_ENABLE = y

LOG_GROUP_START(Optimization_Model)
LOG_ADD(LOG_FLOAT, T_x, &solution[0])
LOG_ADD(LOG_FLOAT, T_y, &solution[1])
LOG_ADD(LOG_FLOAT, T_z, &solution[2])
// LOG_ADD(LOG_FLOAT, Z_AVG, &z_final_measurement)

// LOG_ADD(LOG_FLOAT, Out_d, &euclidean_distance_xy)
// LOG_ADD(LOG_FLOAT, Out_z, &euclidean_distance_z)

// LOG_ADD(LOG_UINT8, Coil_1_sat, &idSaturations[0])
// LOG_ADD(LOG_UINT8, Gial_sat, &idSaturations[1])
// LOG_ADD(LOG_UINT8, Grig_sat, &idSaturations[2])
// LOG_ADD(LOG_UINT8, Ros_sat, &idSaturations[3])

// LOG_ADD(LOG_FLOAT, KF_x, &LKF_ESTIMATION_DEBUG[0])
// LOG_ADD(LOG_FLOAT, KF_y, &LKF_ESTIMATION_DEBUG[1])
// LOG_ADD(LOG_FLOAT, KF_z, &LKF_ESTIMATION_DEBUG[2])

// LOG_ADD(LOG_FLOAT, out, &outlier)

LOG_GROUP_STOP(Optimization_Model)

// PARAM_GROUP_START(Opt_Model_Param)
// PARAM_ADD(PARAM_FLOAT, Opt_STD, &Optimization_Model_STD)
// PARAM_GROUP_STOP(Opt_Model_Param)

LOG_GROUP_START(Dipole_Model)

LOG_ADD(LOG_FLOAT, M_V1, &MeasuredVoltages_calibrated[0])
LOG_ADD(LOG_FLOAT, M_V2, &MeasuredVoltages_calibrated[1])
LOG_ADD(LOG_FLOAT, M_V3, &MeasuredVoltages_calibrated[2])
LOG_ADD(LOG_FLOAT, M_V4, &MeasuredVoltages_calibrated[3])

LOG_GROUP_STOP(Dipole_Model)

PARAM_GROUP_START(Dipole_Params)
PARAM_ADD(PARAM_UINT32, calibTic, &currentCalibrationTick)
// PARAM_ADD(PARAM_FLOAT, STD_nelder, &Optimization_Model_STD)

// initial position
PARAM_ADD(PARAM_UINT8, start0, &initializationOnOrigin)

PARAM_GROUP_STOP(Dipole_Params)