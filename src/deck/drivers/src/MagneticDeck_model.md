# Modello Fisico del Magnetic Deck

## Panoramica del Sistema
Il Magnetic Deck implementa un sistema di localizzazione magnetica basato su 4 bobine posizionate come ancore per determinare la posizione 3D di un tag magnetico montato sul drone Crazyflie.

## Modello del Campo Magnetico

### Dipolo Magnetico
Il sistema modella ogni bobina trasmittente come un **dipolo magnetico** con momento magnetico:
```
m = N × S × I × û
```
Dove:
- **N** = 5 spire per bobina
- **S** = π × r² = superficie bobina (r = 0.019m)
- **I** = 0.5A corrente
- **û** = versore orientamento tag (dal filtro EKF)

### Campo Magnetico del Dipolo
Il campo magnetico **B** generato dal dipolo alla distanza **r** è:
```
B = (μ₀/4π) × (1/r³) × [3(m·r̂)r̂ - m]
```
Dove:
- **μ₀** = 1.25663706212×10⁻⁶ H/m (permeabilità magnetica)
- **r** = vettore posizione tag-ancora
- **r̂** = versore normalizzato tag-ancora
- **r** = distanza euclidea tag-ancora

### Conversione in Tensione Misurabile
La tensione indotta nella bobina ricevente (tag) è:
```
V = G × |2π × f × π × r² × N × (B·n̂)|
```
Dove:
- **G** = guadagno totale sistema (INA + Op-Amp + Potentiometro)
- **f** = frequenza risonanza specifica per ogni bobina
- **n̂** = versore orientamento bobina ricevente
- **B·n̂** = prodotto scalare (componente normale del campo)

## Configurazione Ancore
Coordinate spaziali delle 4 bobine (in metri):
- **Bobina 1 (181kHz)**: (-0.295, +0.25, +0.25) - ROSSO
- **Bobina 2 (189kHz)**: (-0.295, -0.25, +0.25) - GRIGIO  
- **Bobina 3 (210kHz)**: (+0.295, +0.25, +0.25) - NERO
- **Bobina 4 (199kHz)**: (+0.295, -0.25, +0.25) - GIALLO

## Triangolazione e Ottimizzazione

### Funzione di Costo
L'algoritmo Nelder-Mead minimizza:
```
Cost = Σᵢ(Vᵢ_predicted - Vᵢ_measured_calibrated)²
```
Per ogni ancora i = 1,2,3,4 (o 1,2,3 in caso saturazione)

### Processo di Ottimizzazione
1. **Input**: tensioni misurate calibrate, orientamento tag, posizione iniziale
2. **Predizione**: calcolo tensioni teoriche per posizione candidata
3. **Minimizzazione**: ricerca posizione che minimizza errore quadratico
4. **Output**: coordinate 3D ottimali (x,y,z)

## Calibrazione del Sistema

### Acquisizione Riferimento
- **Posizione nota**: (0, 0, 0.01+offset) metri
- **Campioni**: 2000 misure per stabilità statistica
- **Calcolo fattori**: rapporto tensione_misurata/tensione_teorica per ogni bobina

### Correzione Misure
```
V_calibrated[i] = V_measured[i] / calibrationGain[i]
```

## Gestione Orientamento
- **Estrazione**: matrice rotazione 3x3 dal filtro EKF Kalman
- **Conversione**: estrazione versore orientamento tag dal quaternion
- **Utilizzo**: orientamento influenza sia campo B che tensione indotta

## Robustezza e Limitazioni

### Gestione Saturazione ADC
- **Soglia**: tensioni > 1.2V considerate saturate  
- **Fallback**: switch da 4 ancore a 3 ancore automatico
- **Mantenimento precisione**: algoritmo adattivo

### Singolarità Matematiche
- **Origine**: aggiunta offset z=0.0001m se posizione = (0,0,0)
- **Distanza minima**: evita divisioni per zero nel modello 1/r³

### Precisione del Modello
- **Approssimazione dipolo**: valida per distanze >> dimensioni bobina
- **Effetti di bordo**: trascurati (bobine piccole vs distanze operative)
- **Campo uniforme**: assunto all'interno del tag ricevente
