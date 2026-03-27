# Report Tecnico RTF Gazebo - Indagine LiDAR e Bridge ROS

Data: 2026-03-27

Autore test: GitHub Copilot

Repository: /home/ubuntu/Multi-Robot-Nav

## 1) Obiettivo
Verificare in modo quantitativo se il calo del Real Time Factor dipende principalmente da:
1. Configurazione LiDAR GPU (frequenza e visualizzazione)
2. Bridge PointCloud2 da Gazebo a ROS

e documentare una procedura ripetibile per confrontare server e laptop Intel.

## 2) Sintesi esecutiva
Risultato chiave:
1. Baseline (LiDAR 50 Hz, visualize true, bridge scan+points): RTF medio 0.0462
2. Scenario B (LiDAR 10 Hz, visualize false, bridge scan+points): RTF medio 0.6025
3. Scenario C (come B, ma senza bridge PointCloud2): RTF medio 0.6937

Incrementi:
1. Scenario B vs baseline: circa 13.04x
2. Scenario C vs baseline: circa 15.02x
3. Scenario C vs Scenario B: +15.1%

Interpretazione:
1. Il collo di bottiglia principale e misurabile e il LiDAR in configurazione pesante.
2. Il bridge PointCloud2 contribuisce in modo ulteriore e significativo.
3. Sul server testato restano warning EGL su /dev/dri, quindi persiste anche un problema di accesso render path in ambiente virtualizzato/container.

## 3) Configurazione ambiente testato (server)
Profilo macchina rilevato durante i test:
1. CPU: Intel Xeon 6737P, 64 CPU logiche, hypervisor Microsoft
2. RAM: 62 GiB
3. GPU visibile: NVIDIA L40S
4. OS: Ubuntu 24.04 kernel 6.17.0-19-generic

Nota runtime importante:
1. Warning ricorrenti in log: failed to open /dev/dri/renderD128 e /dev/dri/card1 (Permission denied)
2. /dev/dri appartiene a gruppo GID 1003
3. Utente nel container in gruppi 44(video) e 992

Implicazione:
1. Possibile fallback o percorso grafico sub-ottimale lato sensori GPU in VM/container

## 4) Asset e file usati nel test
Script creati per ripetizione test:
1. /home/ubuntu/Multi-Robot-Nav/tools/benchmark/measure_musketeer_rtf.sh
2. /home/ubuntu/Multi-Robot-Nav/tools/benchmark/collect_machine_profile.sh

File di configurazione interessati:
1. /home/ubuntu/Multi-Robot-Nav/clearpath/aramis/robot.urdf.xacro
2. /home/ubuntu/Multi-Robot-Nav/clearpath_ws/src/clearpath_common/clearpath_sensors_description/urdf/velodyne_lidar.urdf.xacro
3. /home/ubuntu/Multi-Robot-Nav/clearpath/aramis/sensors/config/lidar3d_0.yaml

## 5) Metodologia di misura
Metrica:
1. real_time_factor letto da topic /world/warehouse/stats

Procedura per ciascun run:
1. cleanup processi ign gazebo e launch precedenti
2. launch world: ros2 launch musketeers_bringup spawn_world.launch.py world:=warehouse
3. attesa 12 s
4. launch robot: ros2 launch musketeers_bringup spawn_robot.launch.py world:=warehouse generate:=false
5. attesa 25 s
6. campionamento 12 misure da /world/warehouse/stats con ign topic
7. calcolo media, minimo, massimo
8. cleanup finale

Controllo qualità run:
1. Verifica singolo publisher su /world/warehouse/stats durante il campionamento
2. Verifica assenza di processi duplicati world/robot tra run

Perche generate:=false:
1. riduce variabilita dovuta a fase di generazione file e rende i run comparabili

## 6) Scenari testati
### Scenario A - Baseline
Configurazione:
1. update_rate LiDAR = 50
2. visualize = true
3. bridge attivo sia LaserScan sia PointCloud2

Dati campionati:
1. File stats: /tmp/ab_stats_baseline.txt
2. RTF medio: 0.0462
3. Min: 0.0428
4. Max: 0.0528
5. N campioni: 12

Campioni:
~~~
0.0457
0.0453
0.0446
0.0432
0.0428
0.0438
0.0453
0.0457
0.0466
0.0486
0.0504
0.0528
~~~

### Scenario B - LiDAR alleggerito
Configurazione:
1. update_rate LiDAR = 10
2. visualize = false
3. bridge attivo sia LaserScan sia PointCloud2

Dati campionati:
1. File stats: /tmp/lidar10_stats.txt
2. RTF medio: 0.6025
3. Min: 0.4640
4. Max: 0.8365
5. N campioni: 12

Campioni:
~~~
0.5369
0.4963
0.4984
0.6685
0.4987
0.6605
0.8365
0.7387
0.5695
0.7134
0.4640
0.5490
~~~

### Scenario C - LiDAR alleggerito + no PointCloud2 bridge
Configurazione:
1. update_rate LiDAR = 10
2. visualize = false
3. bridge attivo solo LaserScan

Dati campionati:
1. File stats: /tmp/lidar10_nopc_stats.txt
2. RTF medio: 0.6937
3. Min: 0.5725
4. Max: 0.7701
5. N campioni: 12

Campioni:
~~~
0.7386
0.7637
0.7619
0.6611
0.6777
0.5939
0.7504
0.5725
0.7701
0.6649
0.7298
0.6396
~~~

## 7) Evidenze log diagnostiche
Warning EGL presenti in baseline e scenario B:
1. /tmp/ab_world_baseline.log contiene piu linee con:
   - libEGL warning: failed to open /dev/dri/renderD128: Permission denied
   - libEGL warning: failed to open /dev/dri/card1: Permission denied
2. /tmp/lidar10_world.log contiene gli stessi warning

Conferma bridge nello scenario C:
1. /tmp/lidar10_nopc_robot.log mostra solo bridge LaserScan su lidar3d_0
2. assente creazione bridge per scan/points PointCloud2

## 8) Modifiche di configurazione usate nei run
Scenario B (vs baseline):
1. robot.urdf.xacro: update_rate da 50 a 10
2. velodyne_lidar.urdf.xacro: visualize da true a false
3. rebuild package: colcon build --packages-select clearpath_sensors_description

Scenario C (vs B):
1. lidar3d_0.yaml: rimozione blocco bridge PointCloud2

## 9) Perche il laptop Intel puo restare a 80-100% RTF
Ipotesi tecnica forte, coerente con i dati:
1. Sul server VM/container il path rendering sensori GPU non e ottimale (warning EGL + permessi /dev/dri)
2. Il carico LiDAR GPU ad alta frequenza amplifica questo limite
3. Sul laptop Intel il path grafico puo essere piu diretto e stabile (driver Mesa/iGPU nativa, niente penalty VM pass-through comparabile)
4. A parita di codice, differenze driver-stack, virtualizzazione e accesso device grafici spiegano differenze grandi di RTF

Osservazione importante:
1. world-only nel server resta vicino a 1.0
2. crollo avviene soprattutto dopo spawn_robot e attivazione sensori/bridge
3. questo restringe il problema a sensor/render/bridge, non alla fisica base del mondo

## 10) Procedura replicabile sull altro PC
### 10.1 Profilo macchina
~~~
cd /home/ubuntu/Multi-Robot-Nav
tools/benchmark/collect_machine_profile.sh /tmp/profile_other_pc.txt
~~~

### 10.2 Baseline
Prerequisito baseline:
1. update_rate = 50
2. visualize = true
3. bridge PointCloud2 presente in lidar3d_0.yaml

Run:
~~~
cd /home/ubuntu/Multi-Robot-Nav
tools/benchmark/measure_musketeer_rtf.sh baseline generate:=false
~~~

### 10.3 Scenario B
Applicare:
1. update_rate = 10
2. visualize = false
3. colcon build --packages-select clearpath_sensors_description

Run:
~~~
cd /home/ubuntu/Multi-Robot-Nav
tools/benchmark/measure_musketeer_rtf.sh lidar10 generate:=false
~~~

### 10.4 Scenario C
Applicare:
1. rimuovere bridge PointCloud2 da lidar3d_0.yaml (mantenere solo LaserScan)

Run:
~~~
cd /home/ubuntu/Multi-Robot-Nav
tools/benchmark/measure_musketeer_rtf.sh lidar10_nopc generate:=false
~~~

### 10.5 Confronto
Confrontare:
1. medie RTF
2. range min/max
3. presenza warning EGL nei world log
4. publisher stats duplicati (devono essere 1 nel run)

## 11) Checklist anti-falsi positivi
1. Un solo world attivo
2. Un solo spawn_robot attivo
3. Campionare sempre dopo warm-up
4. generate:=false in tutti i run di benchmark
5. Stesso world e stesso robot namespace
6. Stesso numero campioni stats (12)
7. Nessun altro carico GPU pesante in parallelo
8. Acquisire sempre machine profile e log world/robot/stats

## 12) Template risultato per confronto server vs laptop
Compilare questa tabella su entrambi i PC:

| Machine | Scenario | RTF avg | RTF min | RTF max | EGL warning /dev/dri | PointCloud2 bridge |
|---|---|---:|---:|---:|---|---|
| Server VM NVIDIA | Baseline | 0.0462 | 0.0428 | 0.0528 | Si | Si |
| Server VM NVIDIA | LiDAR10 | 0.6025 | 0.4640 | 0.8365 | Si | Si |
| Server VM NVIDIA | LiDAR10 no PC2 | 0.6937 | 0.5725 | 0.7701 | Si | No |
| Laptop Intel | Baseline | TBD | TBD | TBD | TBD | Si |
| Laptop Intel | LiDAR10 | TBD | TBD | TBD | TBD | Si |
| Laptop Intel | LiDAR10 no PC2 | TBD | TBD | TBD | TBD | No |

## 13) Conclusione finale
Il test A/B/C isola chiaramente due contributi principali al degrado:
1. costo del LiDAR GPU ad alta frequenza con visualizzazione
2. costo del bridge PointCloud2

La differenza residua tra macchine, a parita di codice, e molto probabilmente legata al path grafico effettivo e al contesto VM/container, evidenziato dai warning EGL e dal mismatch gruppi device /dev/dri nel server testato.
