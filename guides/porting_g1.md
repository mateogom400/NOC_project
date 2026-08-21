# Porting Go2 → G1 — cosa è cambiato e cosa no

Branch `G1_optimal_trajectory`. Obiettivo: **stesso stack di navigazione
autonoma in ambiente ignoto**, con l'Unitree G1 (umanoide bipede) al posto del
Go2 (quadrupede).

Tesi del porting, verificabile leggendo il diff: **lo stack algoritmico non è
stato riscritto**. Il robot entra nel sistema solo attraverso un file di
parametri e il nome di un topic.

---

## 1. Perché MuJoCo e non Gazebo

Nel Go2 la locomozione in Gazebo è fornita da [`champ`](../src/champ/) +
[`champ_base`](../src/champ_base/) + [`gz_ros2_control`](../src/gz_ros2_control/).
CHAMP è un controllore **quadrupede**: su un bipede non si applica e non esiste
un equivalente drop-in. Per il G1 in Gazebo servirebbe:

- un URDF strumentato con plugin Gazebo (quello disponibile è un URDF puro per
  TF/RViz, senza sensori né `ros2_control`);
- un LiDAR 3D come sensore Gazebo — su una macchina senza GPU, dove il repo già
  gira headless con `LIBGL_ALWAYS_SOFTWARE=1` su llvmpipe;
- soprattutto, **una sorgente di moto**: o una policy RL (che però parla il
  contratto DDS Unitree, non Gazebo), oppure un plugin che teletrasporta la base.

Nell'ultimo caso si arriva allo stesso impianto cinematico che MuJoCo dà già
fatto, dopo giorni di lavoro invece di ore.

Vantaggio metodologico, non solo pratico: l'impianto cinematico di MuJoCo **è**
il modello dell'MPC, quindi il disadattamento modello/impianto è nullo e gli
esperimenti della [roadmap teorica](roadmap_teorica_noc.md) misurano il
comportamento del solutore invece del rumore dell'andatura.

---

## 2. Cosa è stato aggiunto

| Componente | Origine | Note |
|---|---|---|
| [`src/g1_sim/`](../src/g1_sim/) | materiale lab CIHR (`policypilot`) | simulatore MuJoCo ridotto alla sola parte di simulazione |
| `g1_sim/cloud_self_filter.py` | progetto Unitree-G1 | rimozione del rig di sostegno: **no-op in simulazione**, serve sul robot reale |
| [`a_star_mpc_planner/pose_from_tf.py`](../src/a_star_mpc_planner/a_star_mpc_planner/pose_from_tf.py) | progetto Unitree-G1 | sorgente di posa alternativa a `odom_to_pose_node`, con rilevamento di TF congelata |
| [`config/planner_params_g1.yaml`](../src/a_star_mpc_planner/config/planner_params_g1.yaml) | derivato da `Unitree-G1/.../sim/config/mpc.yaml` | ogni valore è marcato `[HW]` (tarato su hardware), `[DER]` (derivato qui) o `[GO2]` (ereditato) |

## 3. Cosa è stato modificato nello stack esistente

Una cosa sola, in cinque file: il topic della posa era cablato a `/go2/pose`.
Ora è il parametro ROS `pose_topic` (default `/robot_pose`), e i profili Go2
esistenti lo impostano esplicitamente a `/go2/pose`, quindi **non si rompe nulla**.

```
a_star_node.py  mpc_node.py  setpoint_to_cmd_vel_node.py
navigation_graph_node.py  odom_to_pose_node.py
```

Nessun'altra riga dell'algoritmo è stata toccata: `mpc_tracker.py`,
`a_star_planner.py`, `gaussian_grid_map.py`, `persistent_map.py` sono identici a
`main`.

## 4. Cosa è stato riusato senza scrivere codice

| Serviva | Riusato |
|---|---|
| nuvola dal frame sensore al frame di pianificazione | [`robot_real_lidar/lidar_filter_node`](../src/robot_real_lidar/) — già parametrico su topic, frame, range, altezze, voxel: solo un YAML |
| `/odom` → `PoseStamped` | `odom_to_pose_node` con un remap |
| setpoint → `/cmd_vel` | `setpoint_to_cmd_vel_node`, solo nuovi clamp |
| `/goal_pose` di RViz → `/global_goal` | [`robot_real_goal_manager/goal_relay_node`](../src/robot_real_goal_manager/) |
| missioni a waypoint ripetibili | `robot_real_goal_manager/mission_runner_node` |

## 4-bis. Rinomina dei pacchetti del percorso di simulazione

I due pacchetti che il G1 riusa sono stati rinominati con `git mv`, in **due
passaggi** che vale la pena raccontare perché il primo era sbagliato:

| originale | primo rename | rename definitivo |
|---|---|---|
| `go2_real_lidar` | `g1_real_lidar` | [`robot_real_lidar`](../src/robot_real_lidar/) |
| `go2_real_goal_manager` | `g1_real_goal_manager` | [`robot_real_goal_manager`](../src/robot_real_goal_manager/) |

Il prefisso `g1` sembrava naturale, ma era **una descrizione falsa**: questi due
pacchetti non contengono una riga di codice specifica di un robot. Topic e frame
sono tutti parametri, e ogni bringup li sovrascrive dal proprio YAML —

| | Go2 | G1 |
|---|---|---|
| `raw_topic` | `/utlidar/cloud` | `/livox/lidar` |
| `source_frame` | `utlidar_lidar` | `mid360_link` |

— quindi sono **adattatori generici**, condivisi fra i due robot. Etichettarli
`g1` produceva l'assurdo di `go2_real_bringup` (bringup hardware del quadrupede)
che dipendeva da `g1_real_lidar`. Il prefisso `robot_`, in linea con `robot_nav`
e `robot_safety` già presenti, dice la verità e sistema entrambi i lati senza
duplicare codice identico.

Rinominati in entrambi i passaggi: directory del package Python interno, marker
in `resource/`, `<name>` in `package.xml`, `package_name` in `setup.py`, percorsi
in `setup.cfg`, entry point, nomi dei nodi ROS (`robot_real_lidar_filter`,
`robot_real_goal_relay`, `robot_real_mission_runner`) e ogni riferimento nel
workspace — zero riferimenti residui.

Attenzione a un punto: il nome del nodo del filtro è anche la **chiave di primo
livello** di `robot_real_lidar/config/lidar_filter.yaml`. Se le due cose divergono
il nodo parte con i default e ignora il file, in silenzio. (`g1_sim` non ha questo
problema perché il suo YAML usa il wildcard `/**`.)

Aggiornati anche i contenuti rimasti specifici del Go2 dentro `robot_real_lidar`,
perché il nome altrimenti mentirebbe: i default passano dal LiDAR L1 del Go2
(`/utlidar/cloud`, frame `utlidar_lidar`) al Mid-360 del G1 (`/livox/lidar`,
frame `mid360_link`), e le altezze di taglio diventano assolute dal pavimento
(0.15–1.60 m) coerenti con un bacino a 0.793 m.

**Effetto collaterale da decidere.** `go2_real_bringup` e `go2_real_planner` sono
il bringup **hardware del Go2**, fuori dal percorso di simulazione, quindi non
sono stati rinominati; i loro riferimenti però sono stati aggiornati ai nuovi
nomi per non lasciare dipendenze rotte. Il risultato è che il bringup del Go2 ora
lancia pacchetti con il prefisso `g1`: incoerente ma funzionante. Le opzioni sono
cancellarli (sono già nell'elenco della §8) oppure rinominarli a loro volta.

## 5. Cosa il G1 rende diverso, e perché

| Aspetto | Go2 | G1 | Motivo |
|---|---|---|---|
| Retromarcia | ammessa | **vietata** | `U[0,k] ≥ 0` era **già** nell'NLP. Sul G1 non è comodità: il Mid-360 copre −7°…+52° in elevazione e ha un cono cieco posteriore, quindi indietreggiare significa muoversi alla cieca |
| Strafe laterale | `vy_max = 0.5` | `vy_max = 0.02` | un bipede non trasla di lato; non 0 esatto per il condizionamento del solutore |
| Inviluppo | 1.0 m/s, 1.5 rad/s | 0.3 m/s, 0.3 rad/s | derating deliberato sotto il tetto della policy AMO (0.5 / 0.4). Tagliare vx più di ω porta il raggio di sterzata minimo da 1.25 a 1.0 m: il robot derated è geometricamente **migliore** nell'evadere |
| Orizzonte | N=50, dt=0.1 (5 s) | N=15, dt=0.20 (3 s) | dt, non N, è la leva: il costo del solve scala con i nodi. Vincolo: una curva a 90° deve stare nell'orizzonte |
| Lag attuatore | τ = 0.12 / 0.10 | τ = 0.001 | in cinematica l'impianto non ha dinamica di attuazione: `lag = 1 − exp(−dt/τ) → 1`. Sotto fisica il lag è reale e va **identificato** ([roadmap §2.1](roadmap_teorica_noc.md)) |
| Inflazione A\* | σ=0.15, τ=0.4 | σ=0.31, τ=0.10 | `d_block = σ·Φ⁻¹(1−τ)`: con ingombro del robot ~0.35 m si vuole d_block ≈ 0.40 m |

## 6. Verifiche fatte

- il modello MuJoCo si carica dagli asset copiati: 31 corpi, 109 geom, 30 giunti;
- il site `mid360` è al posto giusto (origine LiDAR a z = 1.243 m con la base a terra);
- ray-cast del Mid-360 simulato: **8640 raggi in 6.1 ms** → ~164 Hz di soglia,
  cioè il 6 % di un core al rate di 10 Hz. Il rischio CPU che avevo segnalato non
  si materializza;
- tutti i file Python compilano, tutti gli YAML e l'XML sono validi.

**Non ancora verificato**: il run end-to-end sotto ROS (serve il workspace
compilato). È il primo passo successivo.

## 7. Cosa resta da fare

1. `colcon build --packages-select g1_sim a_star_mpc_planner` e primo run;
2. misurare il tempo di solve dell'MPC a N=15/dt=0.20: deve stare sotto i 125 ms
   per reggere gli 8 Hz;
3. rivedere `grid_std` / `obstacle_threshold` se A\* non trova soluzione nei
   corridoi stretti del magazzino;
4. definire alcune coppie start/goal nel magazzino come scenari ripetibili, e
   ri-puntare [`sim_scenarios/`](../src/sim_scenarios/) e
   `run_metrics_collection.sh` al nuovo launch;
5. attaccare il **Blocco A** della [roadmap teorica](roadmap_teorica_noc.md), che
   non dipende dal robot.

## 8. Cosa diventa cancellabile

Con il G1 su MuJoCo non servono più: `champ`, `champ_base`, `champ_msgs`,
`gz_ros2_control`, `go2_description`, `go2_sim`, `sensor_models`, `sim_worlds`,
`d1_sim`, il layer Gazebo di `robot_sim`, e — se non si torna sul Go2 reale —
`go2_bringup`, `go2_real_bringup`, `unitree_api`, `unitree_go`.

Sono lasciati in piedi per ora: la rimozione è una decisione separata, e
`robot_real_lidar` + `robot_real_goal_manager` restano **necessari** perché il G1 li
riusa.
