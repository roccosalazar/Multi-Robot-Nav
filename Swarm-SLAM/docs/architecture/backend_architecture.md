# CSLAM Back-End Architecture Documentation

## 1. Overview

Il **DecentralizedPGO** (Decentralized Pose Graph Optimization) è il nodo back-end del sistema **CSLAM** (Collaborative SLAM). Implementa un algoritmo di ottimizzazione del pose graph distribuito per la localizzazione multi-robot collaborativa.

### 1.1 Obiettivi principali
- Costruire e mantenere un **pose graph locale** per ogni robot
- Scambiare informazioni sul pose graph con i **robot vicini** tramite comunicazione peer-to-peer
- Eseguire **ottimizzazione collaborativa** del pose graph globale aggregato
- **Condividere i risultati** dell'ottimizzazione con tutti i robot connessi
- **Broadcastare i frame di riferimento TF** per la visualizzazione e l'integrazione con il sistema

### 1.2 Dipendenze principali
| Libreria | Scopo |
|----------|-------|
| **GTSAM** | Graph-based Simultaneous Localization and Mapping (ottimizzazione non lineare) |
| **ROS 2** | Framework di comunicazione inter-processo, TF, timer |
| **Boost** | Smart pointer, algoritmi grafici |
| **cslam_common_interfaces** | Message types custom per la comunicazione multi-robot |

---

## 2. Architettura dei Componenti

### 2.1 Struttura del file

```
src/cslam/
├── include/cslam/back_end/
│   ├── decentralized_pgo.h          # Dichiarazioni della classe
│   └── gtsam_utils.h               # Utility di conversione GTSAM ↔ ROS
├── src/back_end/
│   ├── decentralized_pgo.cpp        # Implementazione principale
│   └── gtsam_utils.cpp              # Implementazione utility
└── config/cslam/
    └── example.yaml                 # Configurazione di esempio
```

### 2.2 Classi e Namespace

```
namespace cslam {
    class DecentralizedPGO : public rclcpp::Node {
        // Node principale del back-end
    };
}
```

---

## 3. Flusso di Dati

### 3.1 Diagramma dei Topic

```
┌─────────────────────────────────────────────────────────────────────┐
│                     DecentralizedPGO Node                           │
│                                                                     │
│  INPUT TOPICS:                                                      │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ cslam/keyframe_odom              (KeyframeOdom)           │      │
│  │ cslam/intra_robot_loop_closure   (IntraRobotLoopClosure)  │      │
│  │ cslam/inter_robot_loop_closure   (InterRobotLoopClosure)  │      │
│  │ cslam/optimized_estimates        (OptimizationResult)     │      │
│  │ cslam/get_pose_graph             (RobotIds)               │      │
│  │ /cslam/pose_graph                (PoseGraph)              │      │
│  │ cslam/current_neighbors          (RobotIdsAndOrigin)      │      │
│  └───────────────────────────────────────────────────────────┘      │
│                                                                     │
│  OUTPUT TOPICS:                                                     │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ /r{ID}/cslam/optimized_estimates   (OptimizationResult)   │      │
│  │ /r{ID}/cslam/current_pose_estimate  (PoseStamped)         │      │
│  │ cslam/optimizer_state              (OptimizerState)       │      │
│  │ cslam/get_current_neighbors        (String)               │      │
│  │ /r{ID}/cslam/get_pose_graph        (RobotIds)             │      │
│  │ /cslam/pose_graph                  (PoseGraph)            │      │
│  │ /cslam/viz/pose_graph              (PoseGraph)            │      │
│  │ cslam/heartbeat                    (UInt32)               │      │
│  │ cslam/reference_frames             (ReferenceFrames)      │      │
│  │ cslam/debug_optimization_result    (OptimizationResult)   │      │
│  └───────────────────────────────────────────────────────────┘      │
│                                                                     │
│  TF BROADCAST:                                                      │
│  ┌───────────────────────────────────────────────────────────┐      │
│  │ map_{origin} → map_{local}                                │      │
│  │ map_{origin} → latest_optimized_{local}                   │      │
│  │ latest_optimized_{local} → current_{local}                │      │
│  └───────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2 Diagramma di Sequenza

```
Robot A (Optimizer)          Robot B (Neighbor)         Robot C (Neighbor)
      │                          │                          │
      │  get_current_neighbors   │                          │
      │─────────────────────────>│                          │
      │                          │                          │
      │  current_neighbors       │                          │
      │<─────────────────────────│                          │
      │                          │                          │
      │  get_pose_graph          │                          │
      │─────────────────────────>│                          │
      │                          │                          │
      │  pose_graph              │                          │
      │<─────────────────────────│                          │
      │                          │                          │
      │         [Aggrega pose graphs locali]                │
      │         [Aggiunge prior sul primo pose]             │
      │         [Esegue GNC Optimizer con LM]               │
      │                          │                          │
      │  optimized_estimates     │  optimized_estimates     │
      │─────────────────────────>│─────────────────────────>│
      │                          │                          │
      │  [Applica stimati,       │  [Applica stimati,       │
      │   aggiorna TF]           │   aggiorna TF]           │
      │                          │                          │
```

---

## 4. Stato e Transizioni dell'Optimizer

### 4.1 Enumerazione degli Stati

```cpp
enum class OptimizerState {
    IDLE,                          // 0: Inattivo, in attesa di trigger
    WAITING_FOR_NEIGHBORS_INFO,    // 1: In attesa di informazioni sui vicini
    POSEGRAPH_COLLECTION,          // 2: Raccolta dei pose graph dai vicini
    WAITING_FOR_NEIGHBORS_POSEGRAPHS, // 3: In attesa dei pose graph
    START_OPTIMIZATION,            // 4: Pronto per avviare l'ottimizzazione
    OPTIMIZATION                   // 5: Ottimizzazione in corso (async)
};
```

### 4.2 Macchina a Stati Finiti

```
                    ┌─────────────────────────────────────────────┐
                    │                                            │
                    ▼                                            │
              ┌──────────┐                                      │
              │   IDLE   │                                      │
              └────┬─────┘                                      │
                   │ odometry received                           │
                   ▼                                             │
    ┌──────────────────────────┐                                  │
    │ WAITING_FOR_NEIGHBORS_   │                                  │
    │         INFO             │                                  │
    └──────────┬───────────────┘                                  │
               │ neighbors info received                          │
               ▼                                                  │
    ┌──────────────────────────┐     ┌───────────┐                │
    │   POSEGRAPH_COLLECTION   │────>│   IDLE    │ no neighbors   │
    └──────────┬───────────────┘     └───────────┘                │
               │ request pose graphs                              │
               ▼                                                  │
    ┌───────────────────────────────┐                              │
    │ WAITING_FOR_NEIGHBORS_        │                              │
    │       POSEGRAPHS              │                              │
    └──────────┬────────────────────┘                              │
               │ all graphs received / timeout                     │
               ▼                                                  │
    ┌──────────────────────────┐                                  │
    │   START_OPTIMIZATION     │                                  │
    └──────────┬───────────────┘                                  │
               │ start optimization                               │
               ▼                                                  │
    ┌──────────────────────────┐                                  │
    │      OPTIMIZATION        │                                  │
    └──────────┬───────────────┘                                  │
               │ result ready                                     │
               ▼                                                  │
              ┌──────────┐                                       │
              │   IDLE   │                                       │
              └──────────┘                                       │
```

### 4.3 Logica di Determinazione dell'Optimizer

Il metodo `is_optimizer()` determina se il robot corrente deve eseguire l'ottimizzazione:

```cpp
bool DecentralizedPGO::is_optimizer()
{
    // Priorità basata sull'ID del robot e del suo origin
    bool is_optimizer = true;
    for (unsigned int i = 0; i < current_neighbors_ids_.origins.ids.size(); i++)
    {
        if (origin_robot_id_ > current_neighbors_ids_.origins.ids[i])
        {
            is_optimizer = false;  // Un altro robot ha priorità maggiore
        }
        else if (origin_robot_id_ == current_neighbors_ids_.origins.ids[i] &&
                 robot_id_ > current_neighbors_ids_.robots.ids[i])
        {
            is_optimizer = false;  // Stesso origin, ma ID robot maggiore
        }
    }
    // Deve avere almeno un'odometria
    if (odometry_pose_estimates_->size() == 0)
    {
        is_optimizer = false;
    }
    return is_optimizer;
}
```

**Regola di priorità**: Il robot con il **minore `origin_robot_id`** diventa l'optimizer. In caso di parità, vince il robot con il **minore `robot_id`**.

---

## 5. Struttura del Pose Graph

### 5.1 Rappresentazione GTSAM

Il pose graph è rappresentato usando GTSAM con:
- **Nodi**: `gtsam::LabeledSymbol` con struttura gerarchica
- **Fattori**: `gtsam::BetweenFactor<gtsam::Pose3>` per i vincoli

#### Schema dei Label

```
LabeledSymbol = (GRAPH_LABEL, ROBOT_LABEL(robot_id), keyframe_id)
├── GRAPH_LABEL = 'g'              // Label del grafo
├── ROBOT_LABEL(id) = 'A' + id     // Label del robot (A, B, C, ...)
└── keyframe_id = uint32           // ID del keyframe

Esempio: ('g', 'A', 42) → keyframe 42 del robot 0
```

### 5.2 Tipi di Fattori

| Tipo | Descrizione | Fonte |
|------|-------------|-------|
| **Odometry Factors** | Vincoli consecutivi tra keyframe adiacenti | `odometry_callback()` |
| **Intra-robot Loop Closures** | Vincoli tra keyframe dello stesso robot | `intra_robot_loop_closure_callback()` |
| **Inter-robot Loop Closures** | Vincoli tra keyframe di robot diversi | `inter_robot_loop_closure_callback()` |

### 5.3 Noise Model

```cpp
default_noise_model_ = gtsam::noiseModel::Diagonal::Variances(
    {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});  // 6 DOF: x, y, z, roll, pitch, yaw
```

---

## 6. Pipeline di Ottimizzazione

### 6.1 Fasi dell'Ottimizzazione

```
┌─────────────────────────────────────────────────────────────────┐
│  Fase 1: Raccolta Dati                                         │
│  ─────────────────────                                         │
│  - Ricezione odometria (keyframe_odom)                         │
│  - Ricezione loop closure (intra + inter robot)                 │
│  - Costruzione pose graph locale                               │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Fase 2: Comunicazione                                         │
│  ────────────────────                                          │
│  - Richiesta vicini correnti                                   │
│  - Scambio pose graph con i vicini                             │
│  - Verifica connettività del grafo (BFS)                       │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Fase 3: Aggregazione                                          │
│  ──────────────────                                            │
│  - Unione di tutti i pose graph ricevuti                       │
│  - Filtraggio per connettività                                 │
│  - Aggiunta del prior sul primo pose                           │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Fase 4: Ottimizzazione                                        │
│  ──────────────────                                            │
│  - GNC Optimizer con Levenberg-Marquardt                       │
│  - Esecuzione asincrona (std::async)                            │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  Fase 5: Distribuzione                                         │
│  ───────────────────                                           │
│  - Pubblicazione risultati su /r{ID}/cslam/optimized_estimates │
│  - Aggiornamento TF frames                                     │
│  - Logging (se abilitato)                                      │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 Dettaglio dell'Aggregazione

Il metodo `aggregate_pose_graphs()` esegue:

1. **Verifica connettività** tramite BFS sul grafo di comunicazione
2. **Aggregazione dei valori** (pose estimates) dai robot connessi
3. **Aggregazione dei fattori** (edges) dal grafo locale
4. **Aggregazione dei fattori inter-robot** dai vicini
5. **Deduplicazione** dei loop closures usando un `std::set`

```cpp
std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr>
DecentralizedPGO::aggregate_pose_graphs()
{
    // 1. Check connettività
    auto is_pose_graph_connected = connected_robot_pose_graph();

    // 2. Grafo locale + estimates odometria
    graph->push_back(pose_graph_->begin(), pose_graph_->end());
    estimates->insert(*odometry_pose_estimates_);

    // 3. Estimates dai vicini connessi
    for (auto id : current_neighbors_ids_.robots.ids) {
        if (is_pose_graph_connected[id]) {
            estimates->insert(*other_robots_graph_and_estimates_[id].second);
        }
    }

    // 4. Fattori inter-robot locali
    // 5. Fattori dai grafi dei vicini
    // ... con deduplicazione
}
```

### 6.3 Algoritmo di Ottimizzazione

```cpp
gtsam::Values DecentralizedPGO::optimize(
    const gtsam::NonlinearFactorGraph::shared_ptr &graph,
    const gtsam::Values::shared_ptr &initial)
{
    // GNC = Generalized Concave
    // Usa Levenberg-Marquardt come solver interno
    gtsam::GncParams<gtsam::LevenbergMarquardtParams> params;
    gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
        optimizer(*graph, *initial, params);
    return optimizer.optimize();
}
```

**GNC (Generalized Concave)**: Algoritmo robusto che gestisce automaticamente gli outlier attraverso una funzione di peso concava, riducendo l'impatto dei vincoli errati.

---

## 7. Callback e Timer

### 7.1 Callback dei Topic

| Callback | Topic | Scopo |
|----------|-------|-------|
| `odometry_callback` | `cslam/keyframe_odom` | Riceve keyframe odometry, costruisce fattori odometria |
| `intra_robot_loop_closure_callback` | `cslam/intra_robot_loop_closure` | Riceve loop closure intra-robot |
| `inter_robot_loop_closure_callback` | `cslam/inter_robot_loop_closure` | Riceve loop closure inter-robot |
| `optimized_estimates_callback` | `cslam/optimized_estimates` | Applica i risultati dell'ottimizzazione |
| `get_pose_graph_callback` | `cslam/get_pose_graph` | Risponde a richieste del pose graph |
| `pose_graph_callback` | `/cslam/pose_graph` | Riceve pose graph dai vicini |
| `current_neighbors_callback` | `cslam/current_neighbors` | Riceve lista dei vicini |
| `write_current_estimates_callback` | `cslam/write_current_estimates` | Scrive il grafo corrente in formato G2O |

### 7.2 Timer Periodici

| Timer | Periodo | Callback | Scopo |
|-------|---------|----------|-------|
| `optimization_loop_timer_` | `pose_graph_optimization_loop_period_ms_` | `optimization_loop_callback()` | Loop principale dell'optimizer |
| `optimization_start_timer_` | `pose_graph_optimization_start_period_ms_` | `optimization_callback()` | Trigger iniziale ottimizzazione |
| `tf_broadcaster_timer_` | `pose_graph_optimization_loop_period_ms_` | `broadcast_tf_callback()` | Broadcast TF frames |
| `heartbeat_timer_` | `heartbeat_period_sec_ * 1000` | `heartbeat_timer_callback()` | Heartbeat per discovery |
| `visualization_timer_` | `visualization_period_ms_` | `visualization_callback()` | Pubblicazione per RViz |

---

## 8. Gestione dei Frame di Riferimento (TF)

### 8.1 Gerarchia dei Frame

```
map_{origin_robot}                    ← Frame globale di riferimento
    │
    ├── map_{robot_0}                 ← Frame mappa robot 0
    │   ├── latest_optimized_{robot_0} ← Ultimo pose ottimizzato
    │   │   └── current_{robot_0}     ← Pose corrente (odometria)
    │
    ├── map_{robot_1}
    │   ├── latest_optimized_{robot_1}
    │   │   └── current_{robot_1}
    │
    └── map_{robot_N}
        ├── latest_optimized_{robot_N}
        │   └── current_{robot_N}
```

### 8.2 Definizione dei Frame ID

```cpp
#define MAP_FRAME_ID(id)            "map_" + std::to_string(id)
#define LATEST_OPTIMIZED_FRAME_ID(id) "latest_optimized_" + std::to_string(id)
#define CURRENT_FRAME_ID(id)        "current_" + std::to_string(id)
```

### 8.3 Trasformazioni Broadcastate

Il metodo `broadcast_tf_callback()` pubblica tre trasformazioni:

1. **`map_{origin} → map_{local}`**: Trasformazione tra il frame di origine e la mappa locale
2. **`map_{origin} → latest_optimized_{local}`**: Ultimo pose ottimizzato nel frame globale
3. **`latest_optimized_{local} → current_{local}`**: Differenza odometrica dall'ultima ottimizzazione

```cpp
void DecentralizedPGO::broadcast_tf_callback()
{
    // 1. Origin → Local map
    tf_broadcaster_->sendTransform(origin_to_first_pose_);

    // 2. Origin → Latest optimized pose
    tf_broadcaster_->sendTransform(latest_optimized_pose_msg);

    // 3. Latest optimized → Current (odometry delta)
    gtsam::Pose3 current_pose_diff = local_pose_at_latest_optimization_.inverse() * latest_local_pose_;
    tf_broadcaster_->sendTransform(current_transform_msg);

    // Publish current pose estimate (optimized + odometry)
    optimized_pose_estimate_publisher_->publish(pose_msg);
}
```

---

## 9. Comunicazione Multi-Robot

### 9.1 Protocollo di Scambio Pose Graph

```
┌──────────────────────────────────────────────────────────────┐
│  Richiesta Pose Graph                                        │
│  ────────────────────                                       │
│                                                              │
│  1. Optimizer invia "get_pose_graph" a ogni vicino           │
│     Topic: /r{ID}/cslam/get_pose_graph                       │
│     Payload: RobotIds (lista di tutti i robot coinvolti)      │
│                                                              │
│  2. Ogni vicino risponde con il proprio pose graph           │
│     Topic: /cslam/pose_graph                                 │
│     Payload: PoseGraph (edges + values + connected_robots)    │
│                                                              │
│  3. Optimizer aggrega tutti i grafi ricevuti                 │
│     - Verifica connettività con BFS                          │
│     - Unisce fattori e values                                │
│     - Deduplica i vincoli                                    │
│                                                              │
│  4. Risultati distribuiti a tutti i robot                    │
│     Topic: /r{ID}/cslam/optimized_estimates                  │
│     Payload: OptimizationResult (estimates filtrati per robot)│
└──────────────────────────────────────────────────────────────┘
```

### 9.2 Verifica della Connettività

Il metodo `connected_robot_pose_graph()` implementa un **Breadth-First Search** per determinare quali robot sono connessi al grafo globale:

```cpp
std::map<unsigned int, bool> DecentralizedPGO::connected_robot_pose_graph()
{
    // BFS dal robot corrente
    // Visita tutti i robot raggiungibili tramite inter-robot loop closures
    // Restituisce mappa {robot_id: is_connected}
}
```

Solo i robot connessi contribuiscono all'ottimizzazione globale, garantendo che il sistema funzioni anche con topologie di rete parziali.

### 9.3 Gestione del Timeout

```cpp
bool DecentralizedPGO::check_waiting_timeout()
{
    if ((node_->now() - start_waiting_time_) > max_waiting_time_sec_)
    {
        end_waiting();
        optimizer_state_ = OptimizerState::IDLE;
        RCLCPP_INFO(node_->get_logger(), "Timeout: (%d)", robot_id_);
    }
    return is_waiting();
}
```

---

## 10. Utility GTSAM ↔ ROS

### 10.1 Funzioni di Conversione

| Funzione | Direzione | Tipo |
|----------|-----------|------|
| `odometry_msg_to_pose3()` | ROS → GTSAM | `nav_msgs::msg::Odometry` → `gtsam::Pose3` |
| `transform_msg_to_pose3()` | ROS → GTSAM | `geometry_msgs::msg::Transform` → `gtsam::Pose3` |
| `gtsam_pose_to_msg()` | GTSAM → ROS | `gtsam::Pose3` → `geometry_msgs::msg::Pose` |
| `gtsam_pose_to_transform_msg()` | GTSAM → ROS | `gtsam::Pose3` → `geometry_msgs::msg::Transform` |
| `gtsam_values_to_msg()` | GTSAM → ROS | `gtsam::Values` → `vector<PoseGraphValue>` |
| `values_msg_to_gtsam()` | ROS → GTSAM | `vector<PoseGraphValue>` → `gtsam::Values` |
| `gtsam_factors_to_msg()` | GTSAM → ROS | `NonlinearFactorGraph` → `vector<PoseGraphEdge>` |
| `edges_msg_to_gtsam()` | ROS → GTSAM | `vector<PoseGraphEdge>` → `NonlinearFactorGraph` |

### 10.2 Schema dei Message Types

#### PoseGraphValue
```
uint32 robot_id          # ID del robot
uint32 keyframe_id       # ID del keyframe
float64[7] pose         # [x, y, z, qw, qx, qy, qz]
```

#### PoseGraphEdge
```
uint32 robot0_id         # Robot sorgente
uint32 keyframe0_id      # Keyframe sorgente
uint32 robot1_id         # Robot destinazione
uint32 keyframe1_id      # Keyframe destinazione
float64[7] transform    # Trasformazione relativa
float64[6] covariance   # Covarianza 6x6 (solo diagonale)
```

---

## 11. Configurazione

### 11.1 Parametri Principali

| Parametro | Tipo | Default | Descrizione |
|-----------|------|---------|-------------|
| `robot_id` | `uint32` | - | ID univoco del robot |
| `max_nb_robots` | `uint32` | - | Numero massimo di robot nel sistema |
| `pose_graph_optimization_start_period_ms` | `uint32` | - | Periodo trigger ottimizzazione |
| `pose_graph_optimization_loop_period_ms` | `uint32` | - | Periodo loop principale |
| `max_waiting_time_sec` | `Duration` | - | Timeout attesa vicini |
| `heartbeat_period_sec` | `Duration` | - | Periodo heartbeat |
| `enable_broadcast_tf_frames` | `bool` | - | Abilita broadcast TF |
| `enable_pose_timestamps_recording` | `bool` | - | Registra timestamp dei pose |
| `enable_gps_recording` | `bool` | - | Registra dati GPS |
| `enable_simulated_rendezvous` | `bool` | - | Simula rendezvous |
| `enable_logs` | `bool` | - | Abilita logging dettagliato |
| `log_folder` | `string` | - | Cartella per i log |

---

## 12. Logging e Debug

### 12.1 Debug Pipeline

Il codice include estesi log di debug con il tag `[DEBUG_BACKEND_PIPELINE]`:

```cpp
RCLCPP_INFO(node_->get_logger(),
    "[DEBUG_BACKEND_PIPELINE] keyframe_odom received count=%lu id=%u ...",
    odom_cb_count, msg->id, ...);
```

### 12.2 Logger Integrato

Se `enable_logs_` è attivo, il `Logger` registra:
- Timestamp dei pose
- Pose graph iniziale e ottimizzato
- Informazioni sui grafi ricevuti
- Tempi di ottimizzazione

### 12.3 Output G2O

Il callback `write_current_estimates_callback()` permette di esportare il pose graph corrente in formato G2O per analisi esterna.

---

## 13. Gestione degli Errori

### 13.1 Pattern di Gestione

| Scenario | Gestione |
|----------|----------|
| Ottimizzazione fallita | Catches exception, restituisce initial estimates |
| Timeout attesa vicini | Reset a IDLE, log di avviso |
| Grafo non connesso | Filtra robot non connessi, ottimizza solo la componente connessa |
| Estimates vuoti | Log di warning, ignora il messaggio |
| Scrittura log fallita | Catches exception, log di errore |

### 13.2 Robustezza

- **GNC Optimizer**: Gestisce automaticamente gli outlier
- **BFS Connettività**: Funziona con topologie parziali
- **Timeout**: Previene deadlock in caso di comunicazione interrotta
- **Async Optimization**: Non blocca il loop principale

---

## 14. Estensioni e Personalizzazioni

### 14.1 Punti di Estensione

1. **Strategia di Priorità**: Modificare `is_optimizer()` per criteri diversi dall'ID
2. **Noise Model**: Personalizzare `default_noise_model_` per sensori diversi
3. **Solver**: Sostituire GNC con Dogleg, LM, o altri solver GTSAM
4. **GPS Integration**: Estendere il supporto GPS (già parzialmente implementato)
5. **Simulated Rendezvous**: Estendere la simulazione degli incontri tra robot

### 14.2 Interfacce per il Frontend

Il back-end si aspetta che il frontend pubblichi:
- Keyframe odometry su `cslam/keyframe_odom`
- Loop closure intra-robot su `cslam/intra_robot_loop_closure`
- Loop closure inter-robot su `cslam/inter_robot_loop_closure`

---

## 15. Riferimenti

- **GTSAM Documentation**: https://gtsam.org/
- **ROS 2 Documentation**: https://docs.ros.org/
- **Paper CSLAM**: [Riferimento al paper originale del sistema]
