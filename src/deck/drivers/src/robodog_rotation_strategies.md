# Strategie per la Rotazione del Robodog nel Sistema di Localizzazione Magnetica

## Problema
Quando il robodog ruota, la localizzazione magnetica del Crazyflie si rompe perché le coordinate delle bobine/ancore sono considerate fisse nel modello, mentre nella realtà cambiano.

## Strategie Principali

### 1. Aggiornamento Dinamico del Modello delle Ancore
- Integra l’odometria o IMU del robodog per calcolare in tempo reale posizione e orientamento delle bobine.
- Aggiorna le coordinate delle bobine nel modello magnetico ad ogni ciclo di stima.
- Mantiene la localizzazione corretta anche se il robodog ruota o si muove.

### 2. Allineamento del Crazyflie all’Orientamento del Robodog
- Il drone mantiene il suo orientamento relativo al robodog (es. controllo di yaw).
- Utile solo se il robodog ruota lentamente e il drone può inseguire l’orientamento.

## Altre Strategie Possibili

### 3. Sistema di Riferimento Globale
- Usa coordinate globali (es. motion capture, GPS indoor) per tracciare sia robodog che drone.
- Trasforma le posizioni delle bobine e del drone nel sistema globale per la triangolazione magnetica.

### 4. Sensori di Orientamento sulle Bobine
- Monta IMU (accelerometri/giroscopi) direttamente sulle bobine/ancore.
- Trasmetti l’orientamento delle bobine al sistema di localizzazione.

### 5. Calibrazione Automatica
- Routine che rileva la nuova posizione/orientamento delle bobine quando il robodog si muove o ruota.
- Può essere manuale (trigger da utente) o automatica (rilevamento movimento).

### 6. Fusione Sensoriale
- Combina dati magnetici con altri sensori (UWB, visione, lidar) per correggere errori dovuti a movimenti/rotazioni del robodog.

## Sintesi
La soluzione più robusta è aggiornare in tempo reale la posizione e l’orientamento delle bobine nel modello magnetico, sfruttando odometria, IMU o altri sensori sul robodog. Se il robodog è dotato di odometria affidabile, questa è la via più semplice e precisa. Se non è possibile, considera sensori aggiuntivi o una routine di calibrazione automatica.
