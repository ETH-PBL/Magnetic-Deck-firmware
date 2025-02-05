/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"
#include "estimator.h"

#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"

#include "cf_math.h"
#include "MagneticDeck.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f; // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;



NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t dev;


// -------------- Parametri configurabili --------------
// 1) Altezza del robot (offset da sottrarre quando il drone è fuori)
static float robodogOffset_adjustable = robodogOffset; // h_robot
static float target_fly_height =TARGET_FLYING_HEIGHT; // h_target
static float derivative_threshold_z = 0.05f; // derivata troppo alta

// -------------- Parametri e definizioni --------------

// Variabili globali per la compensazione
static uint8_t state_zone_cf = 0;        // 0 = salita, 1 = discesa , 2 = stazionamento, 3 = derivata troppo alta
static float derivative_z = 0.0f;        // derivata della misura
static bool first_measure = true;        // prima misura
static float originalDistance = 0.0f;  // misura raw (m)
static float compensatedDist  = 0.0f;  // misura compensata
static float raw_measure_t0 = 0.0f;    // misura raw al tempo t0

static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  VL53L1_RangingMeasurementData_t rangingData;
  uint8_t dataReady = 0;
  uint16_t range;

  while (dataReady == 0)
  {
    status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
    vTaskDelay(M2T(1));
  }

  status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
  range = rangingData.RangeMilliMeter;

  VL53L1_StopMeasurement(dev);
  status = VL53L1_StartMeasurement(dev);
  status = status;

  return range;
}

void zRanger2Init(DeckInfo *info)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
    DEBUG_PRINT("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void *arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53L1_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  int measurmentCounter = 0;


  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    point_t cfPosP;
    estimatorKalmanGetEstimatedPos(&cfPosP);

    range_last = zRanger2GetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT)
    {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f + expf(expCoeff * (distance - expPointA)));

      // Logga la misura originale
      originalDistance = distance;

      if (first_measure)
      {
        raw_measure_t0 = distance;
        first_measure = false;
      }
      else
      {
        derivative_z = distance - raw_measure_t0;
        // se la derivata è positiva, il drone sta salendo 
        // quindi non applico la compensazione  
        if ( derivative_z >= 0 && distance < target_fly_height)
        {
          state_zone_cf = 0;
          compensatedDist = distance;
          // aggiorno la misura di riferimento
          raw_measure_t0 = distance;
        }
        // se la derivata supera una certa soglia, il drone sta vedendo il gradino
        // quindi "salto" le misure. Sia in salita che in discesa
        else if (fabs(derivative_z)>derivative_threshold_z)
        {
          state_zone_cf = 3;
          // non aggiorno la misura di riferimento
          // la misura compensata rimane uguale
          compensatedDist = compensatedDist;
          raw_measure_t0 = distance;
        }
        //se vedo una misura maggiore della quota target e la derivata è piccola
        // allora il drone ha superato il gradino e posso applicare la compensazione
        else if (distance > target_fly_height && fabs(derivative_z) < derivative_threshold_z)
        {
          state_zone_cf = 2;
          compensatedDist = compensatedDist + derivative_z;
          // aggiorno la misura di riferimento
          raw_measure_t0 = distance;
        }
        //se la misura è minore della quota target e la derivata è piccola
        // allora il drone è in discesa e non applico la compensazione
        else if (distance < target_fly_height && fabs(derivative_z) < derivative_threshold_z && derivative_z <0)
        {
          state_zone_cf = 1;
          compensatedDist = distance;
          // aggiorno la misura di riferimento
          raw_measure_t0 = distance;
      }else{
        // non dovrebbe mai arrivare qui
        state_zone_cf = 4;
        compensatedDist = distance;
        // aggiorno la misura di riferimento
        raw_measure_t0 = distance;
        DEBUG_PRINT("ERRORE MISURA ZRANGER\n");
        DEBUG_PRINT("MISURA: %f\n", distance);
      }
            
      // Invia la misura compensata all'estimatore
      rangeEnqueueDownRangeInEstimator(compensatedDist, stdDev, xTaskGetTickCount());
      }
        // else: skip se outlier
    }
  }
}

static const DeckDriver zranger2_deck = {
    .vid = 0xBC,
    .pid = 0x0E,
    .name = "bcZRanger2",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_I2C,

    .init = zRanger2Init,
    .test = zRanger2Test,
};

DECK_DRIVER(zranger2_deck);

LOG_GROUP_START(MyZRang)

// Misura originale
LOG_ADD(LOG_FLOAT, original,    &originalDistance)

// Misura compensata
LOG_ADD(LOG_FLOAT, compensated, &compensatedDist)

// Derivata
LOG_ADD(LOG_FLOAT, derivative,  &derivative_z)

// Stato della compensazione
LOG_ADD(LOG_UINT8, state,       &state_zone_cf)



LOG_GROUP_STOP(MyZRang)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck v2](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

/**
 * @brief Offset to subtract when the drone is on the ground
 */
PARAM_ADD_CORE(PARAM_FLOAT, HRobodog, &robodogOffset_adjustable)


/**
 * @brief Target height for the drone to fly at
 */
PARAM_ADD_CORE(PARAM_FLOAT, HTarget, &target_fly_height)
/**
 * @brief Threshold for the derivative term of the measurement
 */
PARAM_ADD_CORE(PARAM_FLOAT, DerivativeThresholdZ, &derivative_threshold_z)


PARAM_GROUP_STOP(deck)
