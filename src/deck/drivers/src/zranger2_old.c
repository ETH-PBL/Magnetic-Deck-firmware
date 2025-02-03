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

static float distanceAfterDetection = 0.0f;
static bool DroneIsOneTheFloor = false;

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t dev;

static float targetFlyingHeight = TARGET_FLYING_HEIGHT; // Valore di default
static float offset_droneOnCenter = 0.05f;               // Valore di default
static float offset_droneOutRobodog = 0.02f;             // Valore di default
static float robodogOffset_adjustable = robodogOffset;                      // Valore di default

// Questi parametri definiscono la sagoma del robot in coordinate (x,y)
static float dogXLimit = 0.26f;  // metà dimensione in X
static float dogYLimit = 0.12f;  // metà dimensione in Y

// Definisci un margine di hysteresis
static float xyMargin = 0.08f;  // 2 cm, ad esempio
static float zMargin  = 0.05f;  // 5 cm, ad esempio

// Margine di transizione (blending) in metri per evitare scatti
// quando il drone esce di poco dalla sagoma.
static float blendMargin = 0.02f;  // 2 cm a titolo di esempio

// Variabili di stato per ricordare se siamo 'fuori' in XY e Z
static bool outsideXYState = false;
static bool outsideZState  = false;
static float compensated = 0.0f;
static float original = 0.0f;

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
      // rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());

      // // detection of the zone of the drone
      // if (distance <= (targetFlyingHeight + offset_droneOnCenter))
      // {
      //   distanceAfterDetection = distance;
      //   // the drone is landed or over the robodog
      //   rangeEnqueueDownRangeInEstimator(distanceAfterDetection, stdDev, xTaskGetTickCount());

      //   DroneIsOneTheFloor = false;
      // }

      // if (distance > (targetFlyingHeight + robodogOffset_adjustable - offset_droneOutRobodog))
      
      // {

      //   DroneIsOneTheFloor = true;

      //   //  the drone is out of the convex hull of the robodog, the measurement has to be compensated
      //   distanceAfterDetection = distance - robodogOffset_adjustable;
      //   // distanceAfterDetection = distance;
      //   rangeEnqueueDownRangeInEstimator(distanceAfterDetection, stdDev, xTaskGetTickCount());
        
      // }

      // Di default, non compensiamo
      original = distance;
      compensated = distance;

      ///////////////////////////////////////////
      // 3) Aggiornamento stato XY (hysteresis)
      ///////////////////////////////////////////
      // Definiamo:
      //  - insideThresholdX = dogXLimit
      //  - outsideThresholdX = dogXLimit + xyMargin
      // e analoghi per Y
      float absX = fabsf(cfPosP.x);
      float absY = fabsf(cfPosP.y);

      // Se attualmente siamo outside, rientriamo inside
      // solo se torniamo sotto insideThreshold (niente margine).
      if (outsideXYState)
      {
        bool fullyInsideX = (absX < dogXLimit);
        bool fullyInsideY = (absY < dogYLimit);
        // Se siamo COMPLETAMENTE dentro su entrambi gli assi
        if (fullyInsideX && fullyInsideY)
        {
          outsideXYState = false;
        }
      }
      else
      {
        // Se siamo inside, passiamo a outside
        // solo se superiamo outsideThreshold su ALMENO un asse
        bool outsideX = (absX > (dogXLimit + xyMargin));
        bool outsideY = (absY > (dogYLimit + xyMargin));
        if (outsideX || outsideY)
        {
          outsideXYState = true;
        }
      }

      ///////////////////////////////////////////
      // 4) Aggiornamento stato Z (hysteresis)
      ///////////////////////////////////////////
      // La vecchia condizione “outsideZ” era:
      // distance > (targetFlyingHeight + robodogOffset_adjustable - offset_droneOutRobodog)
      // Introduciamo un margine +/- zMargin
      float baseZ = (targetFlyingHeight + robodogOffset_adjustable - offset_droneOutRobodog);

      if (outsideZState)
      {
        // Rientriamo “insideZState = false” se distance < baseZ
        if (distance < baseZ)
        {
          outsideZState = false;
        }
      }
      else
      {
        // Passiamo a outsideZState = true se distance > baseZ + zMargin
        if (distance > (baseZ + zMargin))
        {
          outsideZState = true;
        }
      }

      ///////////////////////////////////////////
      // 5) Applico la logica finale
      ///////////////////////////////////////////
      // Se *entrambe* (outsideXYState && outsideZState) sono vere,
      // allora compenso
      if (outsideXYState && outsideZState)
      {
        compensated = distance - robodogOffset_adjustable;

      }

      // 6) Enqueue.
    {
      bool xInBorderZone = (fabsf(cfPosP.x) >= dogXLimit && fabsf(cfPosP.x) <= dogXLimit + xyMargin);
      bool yInBorderZone = (fabsf(cfPosP.y) >= dogYLimit && fabsf(cfPosP.y) <= dogYLimit + xyMargin);
    
      if (xInBorderZone || yInBorderZone) {
        DroneIsOneTheFloor = false;
        // Saltiamo la misura in questa zona
      } else {
        DroneIsOneTheFloor = true;
        rangeEnqueueDownRangeInEstimator(compensated, stdDev, xTaskGetTickCount());
      }
    }
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

LOG_ADD(LOG_FLOAT, MyZRang, &compensated)
//original
LOG_ADD(LOG_FLOAT, original, &original)
LOG_ADD(LOG_UINT8, used, &DroneIsOneTheFloor)

LOG_GROUP_STOP(MyZRang)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck v2](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)
PARAM_ADD_CORE(PARAM_FLOAT, targetFlyH, &targetFlyingHeight)

PARAM_ADD_CORE(PARAM_FLOAT, offsCfCen, &offset_droneOnCenter)
PARAM_ADD_CORE(PARAM_FLOAT, offsCfOut, &offset_droneOutRobodog)

PARAM_ADD_CORE(PARAM_FLOAT, HRobodog, &robodogOffset_adjustable)

// param for the dog limits
PARAM_ADD_CORE(PARAM_FLOAT, dogXLim, &dogXLimit)
PARAM_ADD_CORE(PARAM_FLOAT, dogYLim, &dogYLimit)

// param for the hysteresis margin
PARAM_ADD_CORE(PARAM_FLOAT, xyMarg, &xyMargin)
PARAM_ADD_CORE(PARAM_FLOAT, zMarg, &zMargin)

// PARAM_ADD_CORE(PARAM_FLOAT, blendMarg, &blendMargin)

PARAM_GROUP_STOP(deck)
