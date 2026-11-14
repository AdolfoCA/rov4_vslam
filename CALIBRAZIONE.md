# Calibrazione — cosa serve per un bag compatibile con AQUA-SLAM

> **Nota del 14/11/2026:** i dati sono ora sull'SSD KODAK (`/media/andrea/KODAK/Andrea/`). I bag del 15/10, 20/10, 23/10, 25/10 (kalibr `sessione2/3`), le prove del 26/10, `prova_pipeline_30-10` e le conversioni `01_November` / `02_November` sono stati eliminati: i percorsi che li citano qui sotto sono storici.


Obiettivo: arrivare a un `.bag` ROS 1 che AQUA-SLAM (`stereo_dvl2`) possa usare come
`Structure_Easy.bag`, più uno yaml con i parametri del **nostro** ROV.

Stato di partenza (verificato su `data/dive02`, 13/10/2026):

| Sensore | Topic registrato | Rate | Stato |
|---|---|---|---|
| IMU | `/bluerov2/imu/data_raw` (+ `mag`, `pressure`, `temperature`) | 100 Hz | OK, timestamp già buoni |
| DVL | `/bluerov2/dvl/report` (`bluerov2_msgs/DVLReport`) | 5 Hz | OK, timestamp all'arrivo |
| Camere | `/bluerov2/multicam/aux_{left,right}/image_raw/compressed` | 30 Hz, 960x540 JPEG | OK, **non calibrate** (`camera_info` a zero) |

Frequenze e formato **non sono bloccanti**: si gestiscono con lo yaml e con lo script di
conversione. Il lavoro vero è la calibrazione, sensore per sensore.

Ordine consigliato: **0 → 1 → 2 → 3 → 4 → 5 → 6 → 7**. Le dipendenze sono indicate
in ogni passo.

---

## 0. Script di conversione ROS 2 → ROS 1 (prerequisito per tutto)

Serve per AQUA-SLAM, ma anche per Kalibr e per `allan_variance_ros`, che sono ROS 1.

- [x] **Per Kalibr (25/10):** `data/calibrazione/stereo_e_imu/02_converti_ros1.py` converte
      immagini (JPEG originali, `CompressedImage`) e IMU in un `.bag` ROS 1 con la libreria
      `rosbags`, senza ROS 1. Stessi nomi di topic dell'originale. Verificato con i lettori di
      Kalibr. I bag stanno sull'SSD, `KODAK/Andrea/kalibr/`.
- [ ] **Per AQUA-SLAM:** stesso script da estendere con DVL, formato delle immagini e
      rettificazione (punti sotto).
- [ ] Convertire MCAP/CDR in `.bag` ROS 1.
- [ ] DVL: `bluerov2_msgs/DVLReport` → `waterlinked_a50_ros_driver/DVL` sul topic **`/dvl/data`**
      (nome fisso nel codice di AQUA-SLAM).
  - `beam_valid` → `valid`, `format` → `form`, `time_delta` → `time` (in ms), covarianza scartata.
  - Annullare la rotazione FLU (x, −y, −z) **oppure** tenerne conto in `T_dvl_camera` (passo 5).
- [ ] Immagini: riscrivere `format` in `"bgr8; jpeg compressed bgr8"`.
- [ ] (Opzionale) Timestamp DVL corretti da `time_of_validity` (vedi passo 6).
- [ ] (Opzionale) `/depth/data` dalla pressione — AQUA-SLAM non lo legge.

Non serve rinominare immagini e IMU: i nomi si mettono nello yaml (`LeftImgTopic`,
`RightImgTopic`, `ImuTopic`).

---

## 1. IMU — parametri di rumore (registrazione da fermo)

**Cosa si ottiene:** noise density e random walk di accelerometro e giroscopio
(`IMU.NoiseGyro`, `IMU.NoiseAcc`, `IMU.GyroWalk`, `IMU.AccWalk`). Servono a Kalibr (passo 4)
e allo yaml di AQUA-SLAM.

**Dove:** fuori dall'acqua va bene, anzi è meglio (niente onde, niente correnti).

Passo passo:
- [x] ROV appoggiato **a terra** su una superficie stabile, non su un tavolo, lontano da
      vibrazioni (pompe, compressori, passaggio di persone).
- [x] Motori **disarmati**.
- [x] ~~Accendere il ROV e aspettare 10–15 min per stabilizzare la temperatura.~~ Non serve:
      il riscaldamento si scarta in analisi (vedi risultato sotto).
- [x] Registrare solo il gruppo `imu`.
- [x] Registrare **almeno 1 ora, meglio 2–3 ore**.
- [x] Durante la registrazione nessuno tocca il ROV.
- [x] Controllare `imu/temperature` nel bag.
- [x] Analisi Allan deviation (script nostro invece di `allan_variance_ros`, stesso metodo).
- [x] Moltiplicare i valori per un fattore di sicurezza (×10) prima di usarli in Kalibr / ORB-SLAM3.

### Risultato (T0, 17/10/2026) ✅

**Registrazione:** `/media/andrea/KODAK/Andrea/imu_fermo` (SSD esterno), 15:11 → 17:30,
**138 min**, 829 145 campioni a 100 Hz, ne mancano 7 (0,001%). ROV a terra, fermo, disarmato.

**Temperatura IMU:** 30 → 57,5 °C. Sale in fretta nei primi 30 min, poi lentamente; quasi
ferma dopo ~100 min. Il bias segue la temperatura: giroscopio z da −0,40 a −0,02 °/s,
accelerometro y da 0 a +0,20 m/s². Per l'analisi si usa solo la **parte calda (60–138 min)**.

**Valori** (parte calda, asse peggiore, poi ×10):

| Parametro | Misurato | **Da usare** | Structure_Easy |
|---|---|---|---|
| `IMU.NoiseGyro` / `gyroscope_noise_density` [rad/s/√Hz] | 7,1e-5 | **7,1e-4** | 1e-3 |
| `IMU.NoiseAcc` / `accelerometer_noise_density` [m/s²/√Hz] | 8,2e-4 | **8,2e-3** | 1e-2 |
| `IMU.GyroWalk` / `gyroscope_random_walk` [rad/s²/√Hz] | 1,1e-5 | **1,1e-4** | 1e-5 |
| `IMU.AccWalk` / `accelerometer_random_walk` [m/s³/√Hz] | 1,9e-4 | **1,9e-3** | 1e-4 |

- **Rumore** (N, letto a τ = 1 s): identico su tutta la registrazione, riscaldamento compreso.
- **Random walk** (K): retta +½ tenuta **sopra** la curva di Allan per τ ≥ 20 s. A tempi lunghi
  la curva sale più ripida di +½ (deriva lenta da temperatura): un fit su un tratto corto
  darebbe circa la metà, troppo ottimista. Con solo 90–138 min i valori sono uguali o più bassi.
- Rumore dello stesso ordine di Structure_Easy; random walk 10–20× più grande (IMU economica,
  senza controllo della temperatura).
- Asse y del giroscopio apparentemente meno rumoroso: è la quantizzazione a 1 mrad/s, non un
  sensore migliore. Per questo si usa l'asse peggiore.
- Sotto τ ≈ 0,05 s le curve si appiattiscono (probabile filtro interno di ArduSub): irrilevante.
- Sono **valori di partenza**: se la SLAM con IMU risulta instabile, sono i primi da ritoccare
  (di solito verso l'alto).

**File** in `data/calibrazione/imu/`:
- `imu_kalibr.yaml` — config IMU per Kalibr (passo 4), pronta;
- `aqua_slam_imu.yaml` — righe `IMU.*` per lo yaml di AQUA-SLAM (passo 7);
- `allan_gyro.png`, `allan_acc.png`, `temperatura_bias.png` — grafici;
- `risultati_allan.txt` — tutti i numeri per asse e per tratto;
- `imu_allan.py` — lo script dell'analisi, per rifarla.

Note:
- 100 Hz e giroscopio quantizzato a 1 mrad/s **non sono bloccanti**. Alzare il rate IMU è una
  modifica lato robot (ArduSub): solo dopo averne parlato.
- I timestamp IMU sono regolari (tempo `time_usec` del sensore + filtro di offset), ma Kalibr
  (31/10) li trova **circa 16 ms in ritardo** sui movimenti veri: vedi passi 4 e 6.

---

## 2. Camere stereo — intrinseci e baseline

**Cosa si ottiene:** fx, fy, cx, cy, distorsione di ciascuna camera, e la trasformazione
sinistra↔destra (baseline → `Camera.bf`).

### Risultato (Kalibr, 31/10/2026) ✅ — coppia stereo, camera destra nuova

Registrazioni del 30/10 in piscina con la pipeline nostra (timestamp di cattura, coppie esatte):
`KODAK/Andrea/Pool/30_October/stereo_1`, `stereo_2`, `stereo_3` (2, 3,5 e 3,5 min; target
fermo, ROV guidato con il joystick per coprire tutta l'immagine), unite in un solo bag ROS 1
(`KODAK/Andrea/kalibr/stereo_30-10.bag`). Copertura controllata prima con
`stereo_e_imu/04_controllo_aprilgrid.py`. Kalibr `kalibr_calibrate_cameras`, pinhole-radtan,
`--bag-freq 2`: 1738 viste estratte, **126 coppie tenute**. File e comando:
`data/calibrazione/stereo_e_imu/kalibr_stereo_30-10/` (`lancia.sh`).

| | sinistra (cam0) | destra (cam1) |
|---|---|---|
| fx, fy [px] | 726,10 / 726,83 (±0,4) | 722,64 / 723,16 (±0,4) |
| cx, cy [px] | 478,90 / 271,69 (±0,5) | 479,85 / 277,16 (±0,5) |
| k1, k2, p1, p2 | −0,2912, 0,1031, −0,00028, −0,00010 | −0,2898, 0,1013, 0,00030, −0,00028 |
| errore di riproiezione (std x / y) | **0,13 / 0,12 px** | **0,14 / 0,13 px** |

- **Baseline 22,02 cm** (±0,15 mm): destra rispetto a sinistra −22,01 / +0,25 / +0,36 cm.
- **Rotazione relativa 2,1°**: 2,0° di beccheggio, 0,56° di imbardata, 0,34° di rollio. I 2,0° di
  beccheggio spiegano lo scarto verticale grezzo di circa −26 px misurato al porto il 27/10
  (726 px · tan 2,0° = 25 px). La stima del 26/10 (4° e 3,7°) era sbagliata.
- Sinistra coerente con la calibrazione del 25/10 (focale e distorsione uguali; cx 478,9 contro
  482,3, cioè 3,4 px: preferire il valore nuovo, copertura migliore).

Camchain (da passare al passo 4 e allo yaml di AQUA-SLAM):
`kalibr_stereo_30-10/stereo_30-10-camchain.yaml`.

### Storia (fino al 26/10/2026)

| Data | Calibrazione | Stato |
|---|---|---|
| 15/10 | OpenCV, piscina, `data/calibrazione/camera/out/` | intrinseci validi per la **sinistra** |
| 23/10 | OpenCV "ibrida" (intrinseci 15/10, estrinseci 23/10), `camera/sessione_23-10_estrinseci/` | **superata per la destra**: camera sostituita il 26/10 |
| 25/10 | Kalibr, solo camera sinistra, `stereo_e_imu/kalibr_camera_sx/` | errore di riproiezione circa 0,18 px (OpenCV 1,6 px) |
| **26/10** | **camera destra sostituita** (la vecchia era poco nitida: problema ottico, dimostrato con lo scambio degli stream il 23/10) | ❌ **da rifare in acqua**, con il target |

Controlli sulla camera nuova (26/10, in aria, `KODAK/Andrea/prova_camere_aria`, `prova_lab_aria`):
- nitidezza destra/sinistra **0,91-0,95** (camera vecchia: 0,35-0,50), impostazioni `v4l2-ctl` identiche;
- montata **circa 4° più inclinata in verticale e circa 3,7° in orizzontale** rispetto alla vecchia
  (scarto verticale grezzo −35 px contro +16 px). L'hardware resta così: la rettificazione lo
  compensa (verificato in aria: scarto 0,0 px dopo la rettifica).
- Il supporto delle camere si sposta quando il ROV viene maneggiato (23/10: da 0,0 a +1,8 px in un
  pomeriggio): **controllo a ogni uscita** con `camera/scripts/08_check_dy.py`.
- Tutti i dettagli: `data/Verifica_ORB/fase2_analisi/RESOCONTO_PORTO_20-10.md` (sezioni 10-11),
  `RESOCONTO_PARTE2_24-10.md`, `data/calibrazione/stereo_e_imu/RESOCONTO_CALIBRAZIONI_26-10.md`.

Passo passo:
- [ ] Verificare che aux_left e aux_right **guardino nella stessa direzione e si
      sovrappongano** abbastanza da essere una coppia stereo.
- [ ] Verificare che i video di calibrazione siano **stati fatti in acqua** (l'oblò piatto
      rifrange: gli intrinseci in aria non valgono).
- [ ] Verificare la **risoluzione dei video**: deve essere quella dello stream (**960x540**),
      con lo stesso ritaglio o scala.
  - Se i video sono a risoluzione nativa, riscalare gli intrinseci.
  - Se lo stream è un ritaglio (non una riduzione) i valori non sono trasferibili.
- [ ] Verificare che i video sinistro e destro siano **sincronizzati** tra loro.
- [x] Calibrare (Kalibr `kalibr_calibrate_cameras`), modello pinhole + radtan (31/10).
- [x] Controllare l'errore di riproiezione: 0,12-0,14 px.
- [ ] Salvare il risultato anche come `camera_info_url` per i nodi camera (oggi K è tutto a zero).
- [ ] Rettifica (R1/R2/P1/P2) dalla nuova calibrazione, per AQUA-SLAM e per lo scarto verticale.

Domande aperte (risolte il 30-31/10): video in acqua, 960x540 come lo stream, coppie sincronizzate
(timestamp di cattura).

---

## 3. (Nessun hardware) Verificare la sincronia hardware delle camere

- [x] **Verificato il 26/10: le camere aux scattano insieme**, sfasamento **sotto 1-2 ms**
      (`frame_sync = 1` su tutte le camere). Misura: oggetto mosso in verticale davanti alle
      camere; lo scarto verticale sinistra-destra non cresce con la velocità (5466 punti in
      movimento, retta piatta: +0,1/+0,3 ms). `KODAK/Andrea/prova_sync/`.
  - Gli scarti di ±15 ms visti nei timestamp di arrivo (e l'ipotesi del 25/10 di camere sfasate
    dopo un riavvio) venivano dal percorso rete/driver, non dagli scatti.
  - Con i timestamp RTP (passo 6) resta tra sinistra e destra uno scarto **costante** di circa
    −8,6 ms, che non è reale: viene dall'ordine di codifica sul computer delle camere. Le coppie
    si formano giuste; per la SLAM si può dare alla coppia il timestamp della sinistra.
- [x] **30/10: causa trovata e risolta.** Timestamp del kernel delle due aux: catturano entro
      **±14 µs**. Lo scarto (−8,6 / −14,8 ms) e i cambi di accoppiamento di un frame del porto
      27/10 vengono dal software Blue Atlas, che ri-campiona ogni camera a 30 Hz con l'orologio
      della Jetson (le camere girano a 30,0028 Hz): ogni circa 355 s ogni camera ripete o butta un
      frame, in momenti diversi. Soluzione: pipeline nostra per le aux (`multicam sync-start`,
      `aux_pipeline:=sync`), con timestamp di cattura e accoppiamento esatto. Dettagli:
      `data/prova_pipeline/RESOCONTO_SINCRONIA_CAMERE_30-10.md`.
  - Dati registrati con Blue Atlas: l'accoppiamento sinistra/destra può essere sbagliato di un
    frame a tratti; verificarlo a finestre con `stereo_e_imu/03_accoppiamento_sx_dx.py`.

---

## 4. Camera ↔ IMU — estrinseci e ritardo temporale (Kalibr)

**Cosa si ottiene:**
- `T_imu_camera` (dov'è la camera rispetto all'IMU);
- **ritardo delle camere rispetto all'IMU** (`timeshift_cam_imu`). Fondamentale: ORB-SLAM3
  non lo stima da solo.

**Dipende da:** passo 0 (conversione), 1 (rumore IMU), 2 (intrinseci).

### Risultato (Kalibr, 31/10/2026) ✅ — entrambe le camere, timestamp di cattura

Registrazioni del 30/10 in piscina, pipeline nostra: `Pool/30_October/stereo_imu_1`, `_2`, `_3`
(target fermo, ROV guidato con il joystick; imu_1 lento con il target sempre visto, imu_2 e imu_3
veloci). Controllate prima con `04_controllo_aprilgrid.py` e `05_controllo_camimu.py`, convertite
una per bag. Kalibr `kalibr_calibrate_imu_camera` con il camchain stereo del passo 2 e `imu.yaml`,
`--bag-freq 15`, `--timeoffset-padding 0.1`, `--imu-delay-by-correlation`, tratti
`--bag-from-to` 13-155 / 2-138 / 1-179 s. File e comando:
`data/calibrazione/stereo_e_imu/kalibr_imu_camera_30-10/` (`lancia.sh`).

Posizione nel frame IMU (x avanti, y sinistra, z su) [cm]:

| | imu_1 | imu_2 | imu_3 | media | misura a mano |
|---|---|---|---|---|---|
| camera sinistra | 19,9 / 13,0 / −22,7 | 20,0 / 12,5 / −22,5 | 20,5 / 12,7 / −22,6 | **20,1 / 12,7 / −22,6** | 20 / 14 / −23 |
| camera destra | 19,8 / −9,0 / −22,1 | 19,9 / −9,5 / −21,9 | 20,2 / −9,3 / −22,0 | **20,0 / −9,3 / −22,0** | |

| | imu_1 | imu_2 | imu_3 |
|---|---|---|---|
| inclinazione verso il basso sx / dx | 29,58° / 27,60° | 29,60° / 27,63° | 29,69° / 27,72° |
| **ritardo** sx / dx (t_imu = t_cam + ritardo) | 15,6 / 15,9 ms | 16,3 / 16,4 ms | 15,5 / 15,6 ms |
| errore di riproiezione (mediana) sx / dx | 0,21 / 0,23 px | 0,24 / 0,25 px | 0,29 / 0,30 px |

- **Ripetibilità:** posizioni entro 0,5 cm, rotazioni entro 0,39°, ritardo entro 0,8 ms. Residui
  normalizzati di giroscopio e accelerometro sotto 1 (coerenti con `imu.yaml`).
- **Ritardo circa 16 ms** (era 82 ms con i timestamp di arrivo: era quasi tutto la latenza delle
  immagini). Ora le camere portano l'istante di cattura, quindi i 16 ms sono il ritardo dei
  timestamp dell'**IMU** (catena ArduSub → MAVLink → PC). Uguale per le due camere entro 0,3 ms.
- L'asse verticale è il meno eccitato nelle registrazioni (il ROV si muove poco in su e in giù),
  ma la quota concorda tra le tre stime entro 2 mm e con la misura a mano entro 0,4 cm.

Camera sinistra (**non rettificata**) → IMU, imu_1 (la registrazione con più viste del target):
```
[[-0.00270, -0.49374,  0.86960,  0.19944],
 [-0.99981, -0.01563, -0.01197,  0.12992],
 [ 0.01950, -0.86947, -0.49360, -0.22721],
 [ 0.0,      0.0,      0.0,      1.0    ]]
```

### Risultato precedente (Kalibr, 25/10/2026) — solo camera sinistra, timestamp di arrivo

Kalibr installato sul PC (ROS Noetic) in `~/rov4_vslam/kalibr_ws/`; procedura e funzionamento in
`data/calibrazione/stereo_e_imu/SCHEMA_KALIBR.md`. Registrazioni del 23/10 in piscina, ROV guidato
con il **joystick** (non mosso a mano): sessione 2 (15:19) e sessione 3 (15:33). Solo la camera
sinistra: la geometria tra le camere si spostava durante le registrazioni.

| | Sessione 2 | Sessione 3 | Misurato a mano |
|---|---|---|---|
| Camera sx nel frame IMU (x avanti, y sx, z su) | 20,5 / 12,6 / −25,1 cm | 20,6 / 12,8 / −23,9 cm | 20 / 14 / −23 cm |
| Inclinazione verso il basso | 29,6° | 29,5° | ~29° |
| Ritardo camere (timestamp di arrivo) | 81,9 ms | 82,2 ms | — |
| Errore di riproiezione (mediana) | 0,32 px | 0,37 px | |

Le due sessioni concordano entro 0,3°, 1,2 cm e 0,2 ms. File:
`data/calibrazione/stereo_e_imu/kalibr_imu_camera_sx/`.

Camera (sinistra, **non rettificata**) → IMU, sessione 2 (inversa di `T_cam_imu` di Kalibr):
```
[[-0.00783, -0.49452,  0.86913,  0.20473],
 [-0.99985, -0.00947, -0.01439,  0.12550],
 [ 0.01535, -0.86911, -0.49437, -0.25069],
 [ 0.0,      0.0,      0.0,      1.0    ]]
```

Da fare:
- [x] **Ritardo ristimato (31/10): circa 16 ms**, con i timestamp di cattura (sopra).
- [x] Compensati i 16 ms nel driver IMU (passo 6), dal 31/10.
- [ ] `T_gyro_c` per AQUA-SLAM: comporre la matrice del 31/10 con `R1` della rettifica della
      nuova calibrazione stereo (camera sinistra rettificata).
- [ ] Se la camera sinistra o il suo supporto vengono toccati, rifare la calibrazione.

Preparazione:
- [ ] Stampare un **aprilgrid** (non scacchiera) e incollarlo su supporto rigido e piatto
      (forex/alluminio), plastificato per l'acqua. Misurare la dimensione reale dei tag.
- [ ] Piscina o vasca, con molta luce (serve esposizione breve, niente mosso).

Registrazione:
- [ ] **Stessa pipeline e stessi parametri dell'immersione**: stessa risoluzione, stesso nodo
      camera, stesso `jitter_buffer_ms`. Il ritardo misurato vale solo per quella configurazione.
- [ ] Gruppi attivi: `imu`, `aux_left`, `aux_right`.
- [ ] **Target fermo, ROV in movimento** (mosso a mano davanti al target).
- [ ] 60–90 s di movimento che ecciti **tutti e 6 gli assi**: rollio, beccheggio, imbardata,
      avanti/indietro, su/giù, destra/sinistra.
- [ ] Movimento deciso ma senza mosso nelle immagini; target **sempre visibile da entrambe**
      le camere.
- [ ] Fare 2–3 registrazioni, per avere un riserva e confrontare i risultati.

Elaborazione:
- [ ] Convertire (passo 0), eseguire `kalibr_calibrate_imu_camera` con intrinseci fissati
      dal passo 2 e rumore dal passo 1.
- [ ] Controllare che le registrazioni diano estrinseci e ritardo simili.
- [ ] Riportare `T_imu_camera` nello yaml e il ritardo in `timestamp_offset_s` del nodo
      camera (o correggerlo in post-processing).

---

## 5. DVL ↔ camera — posizione del DVL

**Cosa si ottiene:** `T_dvl_camera`.

- [x] Misurare la posizione e l'orientazione del DVL rispetto all'IMU (metro) — vedi sotto.
- [x] Combinare con camera↔IMU per ottenere `T_dvl_c` — vedi sotto.
- [x] Convenzione: frame nativo A50 (x avanti, y destra, z giù), come Structure_Easy; la
      conversione (passo 0) riporta le velocità del nostro driver (FLU) in quel frame.
- [ ] Verificare l'allineamento del DVL sui dati (P5, o velocità DVL vs movimento camere).
- [ ] Verificare il `speed_of_sound` impostato sul DVL per l'acqua del giorno di raccolta
      (dolce/salata). Modifica lato robot: solo dopo averne parlato.

### Estrinseci misurati a mano (18/10/2026)

Misure con il metro, **origine = IMU (scheda Navigator)**, assi del ROV: x avanti, y
sinistra, z su (scheda da `SCHEDA_MISURE.md`).

| Sensore | x avanti | y sinistra | z su | Orientamento |
|---|---|---|---|---|
| IMU (Navigator) | 0 | 0 | 0 | scheda in verticale, **già compensata da ArduSub**: in T0 la gravità è tutta su +z → rotazione IMU↔ROV = 0 |
| Camera aux_left (centro lente) | +20 cm | +14 cm | −23 cm | inclinata **~28–30° verso il basso** (a occhio), usato 29° |
| DVL (centro faccia trasduttori) | −9 cm | +3 cm | −32 cm | punta in basso, asse x dell'A50 in avanti, `mounting_rotation_offset` = 0 |

AQUA-SLAM (`underwater_orbslam3_blue_gx5_short.yaml`) legge **`T_gyro_c`** e **`T_dvl_c`**
(`Tbc` è ignorato). Convenzione verificata sui valori di Structure_Easy: matrice
camera → sensore, colonne della rotazione = assi della camera nel frame del sensore,
traslazione = posizione della camera nel frame del sensore. La camera è la **sinistra
rettificata** (incluso R1 della rettificazione stereo, ~1°): guarda 28° sotto l'orizzonte.

```yaml
T_gyro_c: !!opencv-matrix      # camera (sinistra, rettificata) -> IMU
  rows: 4
  cols: 4
  dt: f
  data: [ -0.01983, -0.47019,  0.88234,  0.20000,
          -0.99969, -0.00407, -0.02464,  0.14000,
           0.01518, -0.88256, -0.46996, -0.23000,
           0.0,      0.0,      0.0,      1.0 ]

T_dvl_c: !!opencv-matrix       # camera (sinistra, rettificata) -> DVL (frame A50)
  rows: 4
  cols: 4
  dt: f
  data: [ -0.01983, -0.47019,  0.88234,  0.29000,
           0.99969,  0.00407,  0.02464, -0.11000,
          -0.01518,  0.88256,  0.46996, -0.09000,
           0.0,      0.0,      0.0,      1.0 ]
```

Controllo: nel frame del DVL la camera è 29 cm avanti, 11 cm a sinistra, 9 cm più in alto
(20−(−9), 14−3, −23−(−32)).

| | Traslazione | Rotazione | Ritardo |
|---|---|---|---|
| Camera↔IMU | ✅ **Kalibr 31/10**, entrambe le camere, tre registrazioni entro 0,5 cm; misura a mano entro circa 1 cm | ✅ **Kalibr 31/10** (passo 4), entro 0,4° | ✅ **circa 16 ms** (timestamp IMU in ritardo); compensati nel driver IMU dal 31/10 (passo 6) |
| DVL↔camera | ✅ misurata | ✅ assunta allineata, da verificare | — |

Fatto con Kalibr il 25/10 (passo 4). Resta da aggiornare `T_gyro_c` e `T_dvl_c` nello yaml
con la rotazione di Kalibr e la `R1` della nuova calibrazione stereo.

---

## 6. Timestamp

| Sensore | Oggi | Cosa fare |
|---|---|---|
| IMU | ✅ tempo del sensore + filtro | niente |
| DVL | all'arrivo (TCP); nel bag c'è `time_of_validity` (orologio A50 non sincronizzato) | **post-processing**: applicare a `time_of_validity` lo stesso filtro dell'IMU nello script del passo 0 |
| Camere | ✅ **dal 26/10: timestamp RTP** (`timestamp_source: "rtp"` per aux_left e aux_right in `bluerov2.yaml`) → intervalli di 33,33 ms esatti, jitter 0. **Dal 30/10: pipeline nostra** (default della sessione) → **istante di cattura** in ora reale (`timestamp_source: capture`), orologio della Jetson sincronizzato al PC con chrony, coppie con timestamp identici | ritardo camera-IMU da Kalibr → `timestamp_offset_s` |

Camere:
- [x] **Timestamp RTP nel driver (26/10).** Verifiche fatte prima:
  - cattura dei pacchetti di rete (60 s): l'RTP avanza di **3000 tick (33,333 ms) esatti** per
    frame, nessun frame perso, segue il tempo reale entro pochi ppm; l'arrivo sul PC ha 3-4 ms di
    jitter. `KODAK/Andrea/prova_rtp/rtp_60s.pcap`.
  - con il driver nuovo (40 s): intervalli **33,33 ms esatti**, jitter **0,00 ms** (prima 4-6 ms).
    `KODAK/Andrea/prova_rtp/driver_rtp_40s`.
  - Come funziona: `rtpjitterbuffer mode=none` (PTS dai soli timestamp RTP), PTS portato
    nell'orologio ROS con un offset che segue il ritardo minimo di arrivo e si adatta lentamente
    alle derive (`rtp_offset_drift_us_per_s`). Topic diagnostico per camera:
    `/bluerov2/multicam/<camera>/time_reference` (arrivo e tempo RTP di ogni frame).
  - La ricostruzione dei tempi a posteriori sui bag registrati con i timestamp di arrivo è stata
    provata e scartata (25/10): con quei dati non funziona. Per i bag vecchi restano i timestamp
    di arrivo.
- [x] **Pipeline nostra per le aux e driver aggiornato (30/10)**, non ancora committati.
      Resoconto: `data/prova_pipeline/RESOCONTO_SINCRONIA_CAMERE_30-10.md`.
  - `session_start` avvia da solo la pipeline nostra (`multicam sync-start`, al posto del servizio
    camere di Blue Atlas: `bluerov2_bringup/jetson/aux_sync_pipeline.py`); `session_stop` la ferma
    e spegne la Jetson. Niente all'avvio: riaccendendo la Jetson torna Blue Atlas (il ROV è
    condiviso con un collega).
  - **Orologi:** chrony sul PC (server) e sulla Jetson (client), installato con
    `multicam timesync-setup` (`TOPSIDE_SETUP.md` 2g). Scarto verificato +0,4 ms (misurabile a
    ±1,75 ms sul cavo); controllo: `multicam timesync-check`.
  - **Timestamp di cattura:** la pipeline gira sull'ora reale, quindi il timestamp RTP è l'istante di
    cattura e il driver lo usa direttamente (`timestamp_source: capture`), senza stimare latenze.
    Coppie entro 0,27 ms, 0 passi fuori griglia, camere a 30 Hz esatti. Se l'orologio della Jetson
    non è sincronizzato, il driver lo segnala e torna al metodo `rtp` (latenza minima, salita
    100 µs/s, errore di alcuni ms).
  - Con `aux_pipeline:=blueatlas` lo sfasamento di circa 14 ms resta com'era (scelta di Andrea).
- [x] **Ritardo camera-IMU con Kalibr (31/10): circa 16 ms** (passo 4); t_imu = t_cam + 16 ms,
      cioè i timestamp dell'IMU sono 16 ms in ritardo sui movimenti veri.
- [x] **Compensato nel driver IMU (31/10):** `timestamp_offset_s: -0.016` per `imu_node` in
      `bluerov2.yaml` (vale per tutti i topic dell'IMU, dalla sessione successiva).
- [ ] Bag registrati **prima** (fino al 30/10 compreso): i timestamp IMU sono ancora 16 ms in
      ritardo. Correggerli nella conversione o in AQUA-SLAM se si usano quei bag.
- [ ] Il driver perde ancora qualche frame quando il PC non sta al passo (26/10: 4 buchi in 40 s
      di 2-3 frame). I timestamp restano giusti.
- [ ] Il 26/10 il jitter dei timestamp IMU era 1,3-1,7 ms (23/10: 0,2-0,3 ms): ricontrollare sui
      dati del porto (`stereo_e_imu/01_controllo_tempi.py`).

---

## 7. Yaml e bag per AQUA-SLAM

AQUA-SLAM usato: la versione ROS 1 originale in `~/aqua_slam_ws/src/AQUA-SLAM` (container
`orb_dvl2_ros_noetic`, manuale `~/aqua_slam_ws/src/MANUALE_AQUA_SLAM.md`). **Non** la copia ROS 2 in
`ros2_ws/src/aqua_slam` (con `COLCON_IGNORE`, mai usata).

**Script unico:** `data/aqua_slam/converti_per_aqua.py` (sull'host). Dalla registrazione ROS 2 scrive:
- il bag ROS 1 in `~/aqua_slam_ws/src/AQUA-SLAM/dataset/<nome>.bag` (l'unica cartella che il
  container vede);
- lo yaml `~/aqua_slam_ws/src/AQUA-SLAM/data/bluerov2_rov4.yaml`.

Cosa traduce:
- **IMU** `data_raw`: timestamp anticipati di `--ritardo-imu` (default 16 ms; **0 per le
  registrazioni dal 31/10**, già corrette nel driver).
- **Camere:** rettificate con la calibrazione stereo Kalibr (il nodo ROS 1 non rettifica: il
  flag del launch è ignorato), JPEG qualità 95, su `/bluerov2/multicam/aux_*/image_rect/compressed`.
- **DVL:** `bluerov2_msgs/DVLReport` → `waterlinked_a50_ros_driver/DVL` su `/dvl/data` (topic
  fisso nel codice), velocità riportata nel frame dell'A50 (FRD), fasci invariati.

Yaml generato dalle calibrazioni, partendo da `underwater_orbslam3_blue_gx5_short.yaml`:
- [x] `Camera.width/height/fps` 960 / 540 / 30.
- [x] `Camera.fx/fy/cx/cy` della sinistra rettificata (745,63 / 492,44 / 276,04: la rettifica con
      `alpha=0` taglia i bordi), distorsione 0, `Camera.bf` 164,17 (baseline 22,02 cm) ← passo 2.
- [x] `T_gyro_c` ← passo 4: media delle tre camera-IMU del 30/10, composta con R1 della rettifica.
- [x] `T_dvl_c` ← rotazione da `T_gyro_c`, posizione del DVL misurata a mano (passo 5).
- [x] `IMU.Frequency: 100`, rumore da `data/calibrazione/imu/aqua_slam_imu.yaml` ← passo 1.
- [x] Topic: `/bluerov2/multicam/aux_*/image_rect`, `/bluerov2/imu/data_raw`.
- [x] Primo bag convertito (31/10): `AQUA_SLAM_test_2` (piscina 30/10), controllato con
      `check_bag_aqua.py`: tutto OK, l'unico avviso è un buco del DVL di 0,78 s.
- [ ] Prova di AQUA-SLAM (la fa Andrea).

**Questione aperta — registrare già nel formato di AQUA-SLAM**, per non dover più convertire:
- [ ] DVL: pubblicare (anche) `waterlinked_a50_ros_driver/DVL` su `/dvl/data`, velocità in FRD;
- [ ] camere: pubblicare le immagini rettificate (compresse), con la calibrazione del 31/10;
- [x] IMU: ritardo di 16 ms già corretto nel driver (31/10);
- [ ] resta comunque ROS 2 → ROS 1: senza conversione servirebbe `ros1_bridge` o il port ROS 2
      di AQUA-SLAM (vedi il manuale, sezione "Real-time: ROS 1 o ROS 2").

---

## Giorno di raccolta dati

- [ ] Accendere il ROV **durante la preparazione** (varo, controlli, calibrazione DVL): non
      serve tenerlo acceso a vuoto per un'ora. Il bias dell'IMU cambia mentre l'elettronica
      si scalda (in T0: giroscopio z da −0,40 a −0,12 °/s nei primi 30 min), ma la SLAM stima
      il bias da sola durante la registrazione: basta che la deriva sia lenta, ed è lenta.
- [ ] Calibrazione giroscopio DVL (`dvl_calibrate`) dopo la preparazione, subito prima di
      registrare.
- [ ] Stessa configurazione della registrazione Kalibr (passo 4).
- [ ] `multicam timesync-check` passa (orologio della Jetson sincronizzato), poi `session_start`
      (pipeline nostra per le aux di default); a fine giornata `session_stop`.
- [ ] Gruppi attivi: `imu`, `dvl`, `aux_left`, `aux_right` (+ `tf`, costa niente).
- [ ] DVL con fondo agganciato (`velocity_valid: true`) — controllare durante l'immersione.
- [ ] All'inizio almeno **3 m di traslazione** e qualche rotazione per inizializzare l'IMU
      (`IMUInitTranslation: 3.0`).
- [ ] Movimenti fluidi, buona luce, struttura con texture nel campo visivo.
- [ ] Convertire con lo script del passo 0 ed eseguire AQUA-SLAM con lo yaml del passo 7.
