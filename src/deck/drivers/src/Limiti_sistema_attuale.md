
# Limiti del Sistema Attuale di Localizzazione Magnetica

## Problema Principale
Il frame globale di riferimento del drone è fisso sopra il robodog (punto di partenza/calibrazione/atterraggio). La posizione delle bobine rimane corretta, ma se il robodog ruota, l'orientamento delle bobine rispetto al frame globale cambia. Il modello magnetico continua a usare l'orientamento iniziale, causando errori nella stima della posizione del drone.


## Limiti Identificati
- **Orientamento statico delle ancore**: Il modello assume orientamento fisso delle bobine, ma in realtà può cambiare se il robodog ruota.
- **Assenza di aggiornamento dinamico**: Nessun meccanismo per aggiornare l'orientamento delle bobine in tempo reale.
- **Dipendenza dal sistema di riferimento del robodog**: Se il robodog ruota, la triangolazione magnetica diventa incoerente.
- **Assenza di fusione sensoriale**: Il sistema si basa solo su dati magnetici, senza integrare altri sensori.
- **Calibrazione manuale**: La posizione delle bobine deve essere nota e calibrata manualmente.


## Possibili Strategie di Miglioramento
- Aggiornamento dinamico dell'orientamento delle bobine tramite odometria/IMU del robodog.
- Allineamento del drone all’orientamento del robodog.
- Uso di sistema di riferimento globale (motion capture, GPS indoor).
- Sensori di orientamento sulle bobine.
- Calibrazione automatica.
- Fusione sensoriale con altri sistemi di localizzazione.


## Sintesi
Il limite principale è la staticità dell'orientamento delle bobine nel modello. Per superarlo, è necessario integrare dati di orientamento del robodog (tramite odometria, IMU o altri sensori), così da mantenere la coerenza della localizzazione magnetica anche in presenza di rotazioni.
