# Analisi teorica su due piattaforme — G1 e Go2

Domanda a cui questo documento risponde: **le considerazioni teoriche della
[roadmap](roadmap_teorica_noc.md) valgono anche per il Go2, o andrebbe rifatto tutto?**
E, subordinata: si può portare a referto entrambe le piattaforme *senza* scrivere due
volte la formulazione e due volte tutte le metriche?

Risposta breve: **la formulazione va scritta una volta sola perché è già una sola**; le
metriche vanno riportate una volta sola, su due colonne; e per parecchie voci le due
colonne non sono una ripetizione ma **sono il risultato**.

Documenti collegati: [`porting_g1.md`](porting_g1.md) (cosa è cambiato passando da Go2 a
G1), [`roadmap_teorica_noc.md`](roadmap_teorica_noc.md) (le voci teoriche e il loro stato).

---

## 0. Verdetto, misurato

Il profilo Go2 gira **già oggi** dentro l'infrastruttura di analisi, senza modificare una
riga. Caricato e risolto un ciclo su scenario `narrow_gap`:

| profilo | N | dt | n_var | n_con | esito | iter | J |
|---|---|---|---|---|---|---|---|
| `planner_params_g1.yaml` | 15 | 0.20 | **141** | **156** | ok | 21 | 269.9 |
| `planner_params.yaml` (Go2) | 50 | 0.10 | **456** | **506** | ok | 6 | 128.9 |

Riproducibile con qualunque tool di `viz/`, che accetta tutti `--profile`:

```bash
python3 viz/kkt_analysis.py   --profile src/a_star_mpc_planner/config/planner_params.yaml
python3 viz/horizon_sweep.py  --profile src/a_star_mpc_planner/config/planner_params.yaml
python3 viz/make_results.py   --profile src/a_star_mpc_planner/config/planner_params.yaml \
                              --only classe1 --out viz/out_go2
```

Anche `--out` è già parametrico: i due insiemi di risultati convivono senza sovrascriversi.

---

## 1. Cosa è già identico: la struttura dell'NLP

Stessi stati, stessi ingressi, stessa dinamica, stessi tipi di vincolo, stessi termini di
costo. Il conteggio segue **la stessa formula su entrambe le piattaforme**, verificato:

```
n_var = 6(N+1) + 3N            n_con = 6N + 6 + 4N
                                       └──┬──┘ └┬┘ └┬┘
                                   dinamica   x0   box su U (4 righe/passo)

G1  : n_var = 141 (formula 141)   n_con = 156 (formula 156)   ✓
Go2 : n_var = 456 (formula 456)   n_con = 506 (formula 506)   ✓
```

Questa è la constatazione centrale del documento: **le due piattaforme non sono due
problemi, sono due istanziazioni degli stessi simboli.** Nel report la formulazione
dell'FHOCP si scrive quindi una volta sola, in forma parametrica, e le piattaforme entrano
solo attraverso una tabella di istanziazione (§7).

---

## 2. Cosa differisce, e di che natura è la differenza

Non tutte le differenze sono della stessa specie, e la distinzione conta per capire cosa
sia trasferibile e cosa no.

| grandezza | G1 | Go2 | natura |
|---|---|---|---|
| `mpc_N` / `mpc_dt` | 15 / 0.20 s | 50 / 0.10 s | **progetto** |
| orizzonte | 3.0 s | 5.0 s | progetto |
| `mpc_tau_v` / `mpc_tau_w` | 0.001 / 0.001 s | 0.12 / 0.10 s | **impianto** |
| lag effettivo `1-exp(-dt/τ)` | **1.0000 / 1.0000** | **0.5654 / 0.6321** | impianto |
| `vx` ammissibile | `[0, 0.3]` | `[-1.0, 1.0]`\* | **impianto** |
| `vy_max` | 0.02 m/s | 0.5 m/s | impianto |
| `omega_max` | 0.8 rad/s | 1.5 rad/s | impianto |
| pesi Q, R | a mano (tesi) | **ottimizzati con BO** | **taratura** |
| `mpc_W_obs_sigmoid` | 120 | 4.83 | taratura |
| integratore | mid-point | Euler | progetto |
| simulatore | MuJoCo (G1 29 gdl) | Gazebo/CHAMP | impianto |

\* il segno negativo è ciò che oggi manca nel codice: vedi §5.

Le tre nature si comportano in modo diverso:

- le differenze di **progetto** sono scelte nostre, e confrontarle è esattamente lo scopo
  dello sweep N×dt (§10.10 della roadmap);
- le differenze di **taratura** vivono nello spazio dei pesi, ed è lì che stanno il fronte
  di Pareto (§10.11) e la soglia di biforcazione (§10.5);
- le differenze di **impianto** non sono negoziabili: sono il robot.

### Il lag è la differenza che conta di più

Sul G1 il lag è **degenere**: `1 - exp(-0.2/0.001) = 1.0000` esatto, cioè `v_{k+1} = u_k`
e lo stato di velocità è una copia dell'ingresso. Sul Go2 vale 0.5654 e 0.6321 — una
dinamica del primo ordine vera.

Questo non è un dettaglio numerico: **è l'ipotesi che qualifica diverse conclusioni della
roadmap**, in particolare la §10.9 (il vincolo terminale sul G1 non costa nulla *proprio
perché* il lag è degenere). Sul Go2 quel vincolo vincolerebbe davvero.

---

## 3. «Basterebbe permettere al G1 di indietreggiare e di andare più veloce?»

No, per due motivi indipendenti — ed è importante che siano due, perché il secondo è più
forte del primo.

**Non basterebbe.** Anche azzerando la differenza su U_Σ, restano τ (impianto), N e dt
(141 variabili contro 456) e tutti i pesi. I numeri non coinciderebbero comunque.

**E sarebbe sbagliato.** `vx >= 0` non è una restrizione arbitraria da rimuovere per
convenienza: riflette che il G1 cammina e ha il LiDAR frontale. Allargarlo per far tornare
i conti renderebbe il modello **meno fedele all'impianto** — l'opposto di ciò che un
report di controllo ottimo deve difendere. Il vincolo è un risultato da dichiarare, non un
attrito da eliminare.

Corollario pratico: l'obiettivo non è rendere identici i due problemi. È **riconoscere che
la struttura è già identica** e sfruttarlo nella presentazione.

---

## 4. Cosa gira già oggi, senza modifiche

Tutti i tool di `viz/` prendono `--profile`, e [`common.py:33`](../viz/common.py#L33) legge
lo YAML in modo generico (le chiavi introdotte con il G1 sono lette con `.get()` e valore
di default, quindi lo YAML Go2 più vecchio non rompe nulla).

Copertura immediata, a costo zero:

| tool | voce di roadmap |
|---|---|
| `tests/test_integrators.py` | §3.1 ordine di troncamento |
| `guides/snippets/nlp_structure.py` | §0, §1.5 struttura e sparsità |
| `viz/ad_vs_fd.py` | §4.1 AD contro differenze finite |
| `viz/exact_penalty.py` | §2.2 penalità esatta ℓ¹ |
| `viz/kkt_analysis.py` | §2.1 KKT, LICQ, complementarità |
| `viz/bifurcation_sweep.py` | §5.4 regolarità e biforcazione |
| `viz/horizon_sweep.py` | §1.3 sweep N×dt |
| `viz/pareto_front.py` | §1.8 multi-obiettivo |
| `viz/solver_compare.py` | §2.4 active-set contro interior-point |
| `viz/shooting_compare.py` | §1.5 single contro multiple shooting |
| `viz/control_horizon.py` | §1.6 orizzonte di controllo |
| `viz/robust_constraints.py` | §1.2 vincoli robusti (parte offline) |
| `viz/cost_field.py`, `viz/decision_plane.py` | pannelli 1 e 2 |

In pratica: **i capitoli 2, 4, 5, 6 e gran parte del 7**.

---

## 5. Cosa andrebbe cambiato nel codice (piccolo, ma non cosmetico)

Una cosa sola è sostanziale.

**`vx >= 0` è cablato**, non parametrico —
[`mpc_tracker.py:651`](../src/a_star_mpc_planner/a_star_mpc_planner/mpc_tracker.py#L651):

```python
opti.subject_to(U_free[0, k] >= 0.0)
opti.subject_to(U_free[0, k] <= p_vx_max)
```

L'intervento corretto è introdurre `vx_min` **come parametro CasADi**, esattamente come già
avviene per `p_vx_max`, e *non* sostituire le due righe con un `opti.bounded`. La ragione è
strutturale: due righe separate mantengono `n_con = 6N + 6 + 4N` **bit-identico** fra le
piattaforme, mentre un `bounded` le fonderebbe in una sola riga e farebbe scendere il
conteggio di N, rompendo proprio la formula unica su cui si regge tutto il §1. Con il
parametro, fra G1 e Go2 cambia solo *il numero dentro il bound*.

Due allineamenti minori:

- [`common.py:150`](../viz/common.py#L150) — `reach_time` replica l'ipotesi «non
  indietreggia» nel calcolo dell'insieme raggiungibile;
- [`bag_source.py:43`](../viz/bag_source.py#L43) — il topic della posa è fissato a
  `/robot_pose`, mentre il profilo Go2 dichiara `pose_topic: /go2/pose`. Il *nodo* è già
  parametrico; è solo il lettore di bag a non esserlo.

Nessuno di questi è stato implementato: sono elencati qui come perimetro, non come fatto.

---

## 6. Cosa costa davvero: la bag del Go2

Le classi 2 e 3 del generatore ([`make_results.py`](../viz/make_results.py)) leggono una
rosbag di una navigazione vera. Per il Go2 servirebbe registrarla, e lì il simulatore è
**Gazebo/CHAMP** ([`go2_sim/launch/sim_champ.launch.py`](../src/go2_sim/launch/sim_champ.launch.py)),
non MuJoCo: un ambiente diverso, di cui andrebbe prima verificato che parta ancora su
questa macchina. Le ragioni per cui il G1 è passato a MuJoCo sono nella §1 di
[`porting_g1.md`](porting_g1.md).

**Questo, e non il codice di analisi, è il vero costo dell'operazione.**

---

## 7. Come portare entrambe a referto senza raddoppiare niente

Non unificare il problema: **unificare la presentazione.**

### 7.1 Una formulazione, una tabella di istanziazione

Una sola sezione di formulazione, scritta simbolicamente:

> stato x ∈ ℝ⁶, ingresso u ∈ ℝ³, U_Σ = { vx ∈ [v̲ₓ, v̄ₓ], |vy| ≤ v̄_y, |ω| ≤ ω̄ },
> dinamica con costanti (τ_v, τ_w), costo con pesi (Q, R, W_obs), orizzonte N e passo dt.

Poi **una tabella a due colonne** con N, dt, τ, i limiti di U_Σ, i pesi, e in fondo n_var e
n_con calcolati dalla formula del §1. Dieci righe, e le due piattaforme sono coperte senza
ripetere una sola equazione.

### 7.2 Le metriche si dividono in due gruppi netti

**Una riga, due colonne** — tutto ciò che è offline e costa solo `--profile`:
struttura e sparsità, ordine dell'integratore, AD contro differenze finite, penalità ℓ¹,
KKT sugli scenari sintetici, biforcazione, sweep N×dt, Pareto, interior-point contro
active-set, single contro multiple shooting, orizzonte di controllo.

**Solo G1** — tutto ciò che nasce da una bag: errore di predizione (§10.7), β(k) del
vincolo robusto (§10.16, che *deriva* da quell'errore), analisi KKT lungo la missione
reale, ciclo più impegnativo.

Questa divisione non è una lacuna da nascondere, è un **perimetro da dichiarare**:
l'analisi offline gira su entrambe le istanziazioni, quella su dati reali sulla piattaforma
deployata. Una riga di testo nel report la giustifica.

---

## 8. Perché le due colonne sono un risultato, non una ripetizione

Per parecchie voci, affiancare le piattaforme **rafforza l'argomento** invece di diluirlo.

### AD contro differenze finite (§4.1) — misurato

| | G1 (n = 141) | Go2 (n = 456) |
|---|---|---|
| costo del gradiente AD, in valutazioni di f | **1.09** | **2.36** |
| costo che avrebbero le differenze finite (n+1) | **142** | **457** |

Una colonna sola dice «l'AD conviene». Due colonne **dimostrano il teorema**: il costo
dell'AD in modo inverso resta O(1) e limitato dalla sua costante, mentre quello delle
differenze finite scala con n e più che triplica. Il risultato *ha bisogno* di due punti
per essere visibile.

### Lag degenere contro lag vero (§10.9)

È il caso più forte. Sul G1 il vincolo terminale è gratuito, e la roadmap lo attribuisce
esplicitamente alla degenerazione del lag. Il Go2 fornisce **la controprova sperimentale**
di quella spiegazione. Riportare una sola colonna butterebbe via la conclusione più
interessante di tutta la §10.9.

### U_Σ e il vincolo robusto (§10.16)

La §10.16 conclude che il constraint tightening è inerte sulla bag reale **perché** il G1
sta al limite di U_Σ: con `vx ≥ 0` e `vy_max = 0.02` non può ritrarsi lateralmente. Il Go2
non ha quel limite. È la verifica che manca oggi: se la spiegazione è giusta, sul Go2 il
tightening deve muovere la traiettoria.

### Scaling dell'active set (§10.12)

Il vantaggio dell'interior point misurato sul G1 (3.1× → 49.7×) dovrebbe **crescere** su un
problema 3× più grande, perché è l'active set a scalare peggio con il numero di vincoli.
Due colonne trasformano un rapporto in una tendenza.

---

## 9. Decisioni aperte

1. **Parametrizzare `vx_min`** (§5) — piccolo, ma tocca l'NLP deployato: va fatto
   preservando `n_con` e rieseguendo la regressione (`§10.18` della roadmap).
2. **Quante voci portare a due colonne.** Il minimo con resa massima è la sola *classe 1*:
   costa un comando, nessuna modifica al codice, e copre già la tabella AD/FD del §8.
3. **Se registrare una bag Go2** (§6), che è l'unica voce con un costo reale e sblocca le
   classi 2 e 3.
4. **Un fondi-tabelle** che legga i due `results.json` e li affianchi, se si decide di
   andare oltre il confronto manuale. I tool non vanno riscritti: `--profile` e `--out`
   esistono già.

Raccomandazione: fare 1 e 2, rimandare 3 e 4. Il rapporto fra ciò che si guadagna nel
report e ciò che costa è nettamente a favore delle prime due.
