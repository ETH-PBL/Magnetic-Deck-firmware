# Magnetic Deck - Flusso di Lavoro

## Inizializzazione
1. **Setup Hardware**: GPIO, DMA, ADC, DAC, potentiometro per controllo guadagno
2. **Calibrazione Sistema**: Setup tensione riferimento DAC (V_REF_CRAZYFLIE/2), configurazione guadagno totale
3. **Inizializzazione Algoritmi**: FFT ARM DSP, filtro Kalman lineare, parametri Nelder-Mead per ottimizzazione 3D/4D

## Fase Calibrazione (2000 campioni)
1. **Acquisizione**: Raccolta ampiezze da 4 bobine in posizione fissa (0,0,0.01+offset)
2. **Calcolo Fattori**: Determinazione guadagni calibrazione tramite confronto tensioni teoriche vs misurate
3. **Reset EKF**: Riavvio stimatore Kalman dopo calibrazione

## Funzionamento Regime
### Ciclo Principale (SYSTEM_PERIOD_MS)
1. **Acquisizione DMA**: 2048 campioni ADC via interrupt (ADC_Done flag)
2. **Elaborazione Segnale**: 
   - Conversione Q31→Float32
   - Finestra Flattop anti-leakage
   - FFT ARM DSP
   - Estrazione ampiezze 4 frequenze risonanza
   - Ricostruzione parabolica per precisione picchi

3. **Gestione Saturazione**: Controllo soglia, switch algoritmo 3/4 ancore

4. **Stima Posizione**:
   - Estrazione orientamento da matrice rotazione EKF
   - Ottimizzazione Nelder-Mead (funzione costo: minimizza |V_predicted - V_measured|²)
   - Outlier detection: scarto misure con distanza euclidea >0.5m
   - Invio posizione a stimatore principale

### Modello Fisico
- **Campo Magnetico**: Dipolo magnetico 3D per ogni bobina
- **Conversione**: B-field → tensione considerando orientamento, frequenza, guadagno
- **Calibrazione**: Fattori correttivi per compensare variazioni hardware

## Gestione Errori
- Saturazione ADC → modalità 3 ancore
- Restart automatico ADC se bloccato
- Deviazione standard adattiva per altezza
