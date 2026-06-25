# Filtro di Compensazione Z-Ranger2 (anti-drop quota)

## Scopo
Mitigare il drop improvviso della misura di altezza quando il drone vola oltre il bordo del robodog, compensando i gradini e le discontinuità del sensore ToF.

## Parametri principali
- **robodogOffset_adjustable**: offset altezza robodog
- **target_fly_height**: quota target volo drone
- **derivative_threshold_z**: soglia derivata per rilevare gradini

## Variabili di stato
- **state_zone_cf**: zona attuale (0=salita, 1=discesa, 2=stazionamento, 3=derivata alta, 4=errore)
- **derivative_z**: derivata della misura di quota
- **compensatedDist**: quota compensata inviata all'estimatore

## Logica filtro

1. **Prima misura**: inizializza riferimento quota
2. **Calcolo derivata**: `derivative_z = distanza_attuale - distanza_riferimento`
3. **Zone di compensazione**:
   - **Salita** (`derivative_z >= 0` e quota < target): nessuna compensazione, aggiorna riferimento
   - **Gradino** (`|derivative_z| > soglia`): ignora misura, mantiene quota precedente
   - **Stazionamento sopra gradino** (quota > target e derivata piccola): applica compensazione, aggiorna riferimento
   - **Discesa** (quota < target, derivata piccola e negativa): nessuna compensazione, aggiorna riferimento
   - **Errore**: fallback, aggiorna riferimento e quota

4. **Output**: invia `compensatedDist` all'estimatore, con deviazione standard adattiva

## Robustezza
- Ignora outlier >5m
- Logga stato, quota raw, quota compensata e derivata per debug
- Parametri configurabili runtime via interfaccia
