# Roadmap teorica — copertura del corso *Numerical Optimization for Control* (062047)

Guida di riferimento per portare questo repository dentro il perimetro teorico del corso
062047 (L. Fagiano, PoliMi). Ogni voce è una scheda autocontenuta:

- **Corso** — la sezione esatta delle dispense
- **Stato** — cosa c'è oggi nel repo, con il file
- **Da aggiungere** — l'intervento concreto
- **Verifica** — il numero o la figura che ne dimostra l'effetto
- **Sforzo** — S (ore), M (un giorno), L (più giorni)

Convenzione dei riferimenti: nelle righe **Corso** i `§` rinviano alle *dispense*; altrove
rinviano alle sezioni di questa guida.

## Perimetro: quale problema stiamo risolvendo

Le dispense trattano tre famiglie di problemi di ottimizzazione applicati ai sistemi dinamici
(§7.3 e §3): **identificazione**, **stima dello stato** e **controllo ottimo**. Questo progetto
appartiene alla terza e solo alla terza: risolviamo online un **FHOCP non lineare**
(§3.3 eq. (3.11)) a orizzonte recedente.

Di conseguenza sono **fuori perimetro per scelta**, non per dimenticanza:

- l'identificazione dei parametri del modello con metodo dell'errore di simulazione
  (§3.1.2, §7.3.1) — è *optimal system identification*;
- la stima dello stato con Moving Horizon Estimation (§3.2, §7.3.2) — è *optimal observer*.

Entrambe sarebbero implementabili qui, ed entrambe comparivano nella prima versione di questa
guida. Sono state rimosse perché diluirebbero l'elaborato su tre categorie di problema invece
di approfondirne una. La §8 elenca cosa resta fuori e perché.

Il baricentro è quindi il **Capitolo 7** (soluzione dei problemi di controllo ottimo), che qui
viene per primo, sostenuto dal **Capitolo 6** (ottimizzazione vincolata: è il macchinario con
cui l'FHOCP si risolve), dal **Capitolo 2** (discretizzazione: è come l'FHOCP nasce) e dal
**Capitolo 5** (derivate: è cosa lo rende calcolabile).

---

## 0. Stato di partenza, misurato

NLP effettivamente costruito da [`mpc_tracker.py`](../src/a_star_mpc_planner/a_star_mpc_planner/mpc_tracker.py)
con i parametri deployati sul G1 in
[`planner_params_g1.yaml`](../src/a_star_mpc_planner/config/planner_params_g1.yaml) (N = 15, dt = 0.2 s):

| grandezza | valore |
|---|---|
| variabili decisionali | **141** (X: 6×16 = 96, U: 3×15 = 45) |
| vincoli totali | **156** |
| — di uguaglianza | 96 (6×15 dinamica + 6 condizione iniziale) |
| — di disuguaglianza | 60, **esclusivamente box sugli ingressi** (4 righe per passo) |
| parametri CasADi | 121 |
| densità Jacobiano dei vincoli | 1.60 % (351 nnz) |
| densità Hessiana lagrangiana | 4.30 % (855 nnz) |
| termini ostacolo | 128 sigmoid + 128 hinge², **tutti nel costo, nessuno nei vincoli** |
| orizzonte | 3.0 s ⇒ **0.60 m** di percorso a `v_ref` = 0.2 m/s |

Riproducibile con `guides/snippets/nlp_structure.py` (vedi *Riproducibilità*, §7). Per confronto, il profilo Go2
(N = 50, dt = 0.1) dà 456 variabili, 506 vincoli, densità Jacobiana 0.67 %.

### Quattro fatti misurati che vincolano tutta la guida

**1. Gli ostacoli non sono vincoli, sono penalità.** Non esistono moltiplicatori di ostacolo,
non c'è un active set non banale, e tutto il Capitolo 6 resta inutilizzato. È il buco singolo
più grande.

**2. Il ritardo di attuazione del primo ordine è degenere sul G1.** Il profilo deployato ha
`mpc_tau_v = mpc_tau_w = 0.001` s contro `dt = 0.2` s, quindi

```
lag = 1 − exp(−dt/τ) = 1 − exp(−200) = 1.000000000000   (esattamente 1 in float64)
```

cioè `v_{k+1} = u_k`. Le tre componenti di velocità nello stato **non sono stati dinamici**:
sono copie ritardate di un passo dell'ingresso. Il modello "a 6 stati con lag del primo ordine"
è, come deployato, un **uniciclo a 3 stati con un passo di ritardo sull'ingresso**. Questo
tocca le §3.1, §3.2 e §5.4, e va detto invece che rivendicare un modello che non è quello in
esecuzione.

La degenerazione non è un'inferenza: **si vede nella sparsità**. A parità di N = 50, il profilo
Go2 (τ = 0.12 s) dà 1556 nnz nello Jacobiano dei vincoli, il profilo G1 ne dà 1156. I 400 nnz
mancanti sono esattamente 8 per passo: CasADi elimina simbolicamente la dipendenza di `v_{k+1}`
da `v_k`, perché con `lag = 1` quel coefficiente è nullo. La struttura del problema registra la
degenerazione da sé.

**3. Il paesaggio di costo non biforca ai pesi deployati.** Misurato su due missioni reali del
G1 nel magazzino (`viz/bags/industrial_plant`, `industrial_plant_fix`): partendo da warm start
opposti, sinistra e destra convergono alla **stessa** soluzione, distanza in ℝ¹⁴¹ pari a
0.0000. Con `W_obs_sigmoid = 120` contro `Q_x = Q_y = 200`, l'evitamento ostacoli è di fatto
**delegato ad A\***. La biforcazione compare tra W_obs 300 e 600.

**4. La fattibilità ricorsiva è già stata persa in esercizio, con conseguenze misurate.**
Vedi §1.2: un fallback pensato come transitorio si è rivelato permanente e ha disattivato
l'MPC per i due terzi di una missione. È il materiale sperimentale della §7.2.5 delle dispense,
già in casa.

### Cosa è già stato implementato

Tre voci della roadmap sono già fatte e misurate — integratore RK2 (§3.1), analisi KKT (§2.1),
penalità esatta ℓ¹ (§2.2) — oltre all'infrastruttura di analisi offline in [`viz/`](../viz/).

**Il riepilogo con i risultati e la lista di cosa resta sono nella [§10](#10-stato-di-avanzamento)**,
in fondo a questo documento. Le singole schede qui sotto riportano i numeri nel dettaglio.

---

## 1. Capitolo 7 — Soluzione dei problemi di controllo ottimo *(il baricentro)*

### 1.1 Path following con l'ascissa come variabile decisionale ⭐

- **Corso** — §7.2.4, eq. (7.5). Testualmente: *"la velocità alla quale il sistema riesce a
  seguire il percorso non è nota a priori, quindi una formulazione di inseguimento di
  traiettoria è difficile da impostare, perché si dovrebbe decidere arbitrariamente (o con
  euristiche) l'andamento temporale dei riferimenti"*. La soluzione: rendere gli incrementi
  `Δθ(t)` variabili decisionali con `Δθ ≥ 0`, `θ(0) = 0`, `θ(N) = 1`, e mettere a costo
  `(1 − θ(t))²`.
- **Stato** — **assente, ed è il caso descritto alla lettera**.
  [`mpc_tracker.py::_build_reference`](../src/a_star_mpc_planner/a_star_mpc_planner/mpc_tracker.py)
  campiona il riferimento lungo il path A\* a velocità costante:
  `s_k = min(s0 + v_ref·k·dt, total)`. Con `v_ref = 0.2` m/s scelto a mano, è precisamente la
  scelta arbitraria che il corso raccomanda di eliminare.

  *(Nota: il commento `# (not used)` accanto a `mpc_v_ref` nel vecchio `planner_params.yaml`
  è errato — il parametro è usato eccome, alla riga 454.)*
- **Da aggiungere** — riformulazione (7.5):

  ```
  min  α₁ Σ_k (z̄(θ_k) − z_k)ᵀ Q (z̄(θ_k) − z_k) + α₂ Σ_k u_kᵀ R u_k + α₃ Σ_k (1 − θ_k)²
  s.t. dinamica del robot                     (invariata)
       θ_{k+1} = θ_k + Δθ_k                   (nuova, banale)
       Δθ_k ≥ 0                               (monotonia lungo il path)
       θ_0 = 0,  θ_N ≤ 1                      (disuguaglianza: il path può non finire)
       box su u                               (invariati)
  ```

  `z̄(θ)` è il path A\* parametrizzato: già disponibile, basta sostituire il campionamento a
  passo `v_ref·k·dt` con l'interpolazione in θ. Aggiunge N+1 variabili e N disuguaglianze —
  quindi **cambia anche il bilancio della §2.4**.
- **Perché conta qui in particolare** — con l'orizzonte deployato il robot copre **0.60 m**,
  contro un `mpc_lookahead_dist` di 0.45 m. Il riferimento temporale a velocità fissa è quindi
  tarato su un margine minuscolo: se il G1 rallenta (curva stretta, ostacolo), il riferimento
  gli scappa avanti e il costo di tracking cresce per una ragione che **non ha nulla a che
  vedere con la qualità del controllo**. La parametrizzazione in θ elimina esattamente questo
  artefatto.
- **Verifica** — accuratezza geometrica (distanza dal path, non dal riferimento temporale),
  tempo di completamento ed energia di controllo per diverse terne (α₁, α₂, α₃) ⇒ alimenta
  direttamente la §1.6 (Pareto). Confronto diretto su `viz/bags/industrial_plant_fix`.
- **File** — `mpc_tracker.py::_build_reference` e `::_build_nlp`
- **Sforzo** — **M** · **massimo rapporto rilevanza / dimensione della modifica**

---

### 1.2 Ingredienti terminali e fattibilità ricorsiva ⭐

- **Corso** — §7.2.5. Distinzione fondamentale tra **insieme di fattibilità F** (gli stati z₀
  per cui l'FHOCP ha soluzione) e **insieme ammissibile Ω** (i valori di U ammissibili dato z₀).
  Se la traiettoria parte in F e ne esce, la fattibilità ricorsiva non vale. Il rimedio
  raccomandato per NMPC è il **vincolo terminale di equilibrio**: la coda della sequenza
  precedente più l'ingresso di equilibrio è sempre ammissibile, quindi Ω ≠ ∅. Il corso segnala
  anche che nella pratica i vincoli si implementano **soft**, con slack penalizzati.
- **Stato** — **assente, e sostituito da due euristiche** che sono, letteralmente, gestioni di
  perdita di fattibilità ricorsiva:
  - limiti di velocità ridotti quando il tasso di fallimento IPOPT supera il 30 %
    (`adaptive_vel_limits` in `mpc_node.py`);
  - fallback a velocità nulla dopo `_MAX_CONSEC_FAILURES = 3` solve falliti.
- **Materiale sperimentale già in casa** — la seconda euristica si è rivelata un **latch
  permanente**: il ritorno anticipato precedeva l'unico punto che azzerava il contatore, quindi
  al terzo fallimento consecutivo il fallback si auto-alimentava per il resto della missione.
  Misurato su `viz/bags/industrial_plant`:

  | | con il latch | dopo la correzione |
  |---|---|---|
  | cicli con solve riuscito | 267 / 876 (30 %) | 775 / 775 (100 %) |
  | cicli in cui IPOPT non è mai partito | 606 | 0 |
  | `solve_ms` medio | 25.5 (gonfiato dai salti) | 70.0 |
  | distanza finale dal goal | 0.29 m | 0.18 m |

  Rigiocando la bag attraverso il tracker corretto: **99 % di successi**. Le NLP erano tutte
  risolvibili; nessun fallimento era dovuto a difficoltà numerica. Il robot ha comunque
  raggiunto il goal, **navigando due terzi della missione senza MPC**, sul solo path A\*.
- **Perché è un risultato e non solo un bug** — è la dimostrazione empirica dell'argomento di
  §7.2.5: in assenza di ingredienti terminali che *garantiscano* Ω ≠ ∅, si finisce a scrivere
  fallback ad hoc, e i fallback ad hoc hanno modi di fallimento propri. Il corso lo dice in
  astratto; qui c'è la misura.
- **Da aggiungere** —
  1. **vincolo terminale di equilibrio** `v_N = 0`: il robot si può sempre fermare entro
     l'orizzonte. Sul modello a 6 stati è immediato perché la velocità è nello stato — e resta
     immediato anche nella degenerazione del §0.2, perché equivale a `u_{N−1} = 0`;
  2. **slack sul vincolo terminale**, penalizzato in ℓ¹ (§2.2), così il problema resta sempre
     ammissibile come raccomanda il corso;
  3. in alternativa più ambiziosa, **insieme terminale come sottolivello di una funzione di
     Lyapunov**: `X_f(α) = {x : xᵀPx ≤ α}` con P dalla DARE del problema LQ ausiliario, e α
     scelto come `min_i d_i²/(c_iᵀP⁻¹c_i)` perché `X_f(α)` rispetti tutti i vincoli. È la
     costruzione classica ed è nelle dispense; va però detto che su un modello non lineare
     l'argomento LQ vale solo localmente;
  4. **discussione onesta**: le due euristiche restano nel codice come misure di sicurezza, ma
     vanno presentate come *"ciò che facevamo in assenza di ingredienti terminali"*, con il
     confronto quantitativo prima/dopo — che ora esiste.
- **Verifica** — tasso di fallimenti IPOPT e di attivazione del fallback, con e senza vincolo
  terminale, sulle bag già registrate; percentuale di solve con slack terminale > 0.
- **Sforzo** — **M** (punti 1-2 e 4) · **L** se si fa anche il punto 3

---

### 1.3 Lunghezza dell'orizzonte: prestazione contro tempo di calcolo

- **Corso** — §7.2.5. L'orizzonte è il parametro che governa il compromesso fra qualità
  dell'anticipazione e costo computazionale; la fattibilità in tempo reale è un vincolo di
  progetto, non un dettaglio.
- **Stato** — **N = 15 senza alcuna giustificazione scritta**, ereditato dalla taratura
  hardware del G1.
- **Perché qui è particolarmente critico** — l'orizzonte copre **0.60 m**. Un ostacolo visto a
  1 m è fuori dall'orizzonte di predizione: l'MPC non lo vede proprio, e questo spiega da solo
  il fatto misurato al §0.3 (l'evitamento lo fa A\*). Non è una taratura debole, è una scelta
  strutturale che va o giustificata o cambiata.
- **Da aggiungere** — sweep N ∈ {5, 10, 15, 25, 40, 60} a dt fisso, riportando per ciascuno:
  - metrica di prestazione in anello chiuso (distanza finale, clearance minima, lunghezza del
    percorso, energia di controllo) sulle bag già registrate;
  - distribuzione del tempo di solve (mediana, p95, massimo) — non solo la media: il vincolo
    real-time lo viola la **coda**, e nella nostra bag c'è già un solve da 1393 ms contro un
    p95 di 102 ms;
  - numero di iterazioni IPOPT (ora disponibile su `/mpc/diagnostics`).

  Va discusso anche il compromesso alternativo: allungare l'orizzonte **temporale** alzando
  `dt` invece di N, che non aumenta le variabili ma peggiora l'errore di discretizzazione (§3.1).
- **Verifica** — due grafici: metrica di prestazione vs N, e boxplot del tempo di solve vs N
  con la linea del budget (125 ms a 8 Hz). Il punto scelto va motivato su quei grafici.
- **Sforzo** — **M** · l'infrastruttura di replay c'è già, lo sweep è uno script

---

### 1.4 Modello di predizione contro modello di simulazione

- **Corso** — §7.2.5. L'MPC predice con un modello nominale; l'impianto è un'altra cosa. Il
  disallineamento produce un offset di regime che la sola retroazione a orizzonte recedente
  non annulla.
- **Stato** — **presente nei fatti, assente nella trattazione**. Il modello di predizione è un
  uniciclo (più il lag degenere del §0.2); l'impianto è **MuJoCo con il G1 a 29 gradi di
  libertà** che cammina. Il disallineamento è enorme e completamente non modellato: passi
  discreti, oscillazione laterale del bacino, ritardo del controllore di camminata.
- **Perché è un punto forte e non una debolezza** — è un caso di studio migliore di un
  disturbo additivo sintetico: il mismatch è **reale e strutturale**, non iniettato a mano.
- **Da aggiungere** —
  1. **quantificarlo**: dalle bag, confrontare la traiettoria predetta a k passi
     (`/mpc/predicted_path`, già registrata) con quella effettivamente percorsa
     (`/robot_pose`). Errore di predizione a 1, 5, 15 passi, in funzione della velocità
     comandata e della curvatura. È una misura che si estrae dai dati esistenti, senza
     nuovi esperimenti;
  2. **discutere l'offset**: in un compito di regolazione il mismatch dà errore permanente.
     Qui il goal viene raggiunto (0.18 m) perché A\* ripianifica, cioè l'anello esterno
     compensa quello interno — va detto esplicitamente, perché è il motivo per cui l'offset
     non si vede;
  3. **eventualmente** un'azione integrale sull'errore di tracking, con lo stato aumentato
     `η_{k+1} = η_k + dt·e_k`. È la ricetta standard e costa uno stato.
- **Verifica** — curva dell'errore di predizione vs orizzonte di predizione; confronto con e
  senza azione integrale sulla stessa bag.
- **Sforzo** — **S** (punto 1, solo analisi) · **M** (punto 3)

---

### 1.5 Single shooting contro multiple shooting

- **Corso** — §7.2.2, eq. (7.3) single vs eq. (7.4) multiple, Fig. 7.2. Vantaggio del multiple:
  il modello non viene mai integrato in anello aperto per più di un passo; svantaggio: più
  variabili e vincoli, compensati dalla **sparsità** e da un numero minore di iterazioni.
- **Stato** — multiple shooting **usato** (X e U entrambe variabili decisionali, dinamica
  imposta come uguaglianza), mai dichiarato né motivato né misurato.
- **Da aggiungere** — la versione single-shooting a confronto: eliminando X per sostituzione si
  passa a 45 variabili e nessun vincolo di dinamica, ma con Jacobiana **densa**. Misurare
  entrambe.

  La metà multiple-shooting della tabella è già misurabile (profilo G1, `nlp_structure.py`):

  | N | variabili | vincoli | jac nnz | densità jac | densità hess |
  |---|---|---|---|---|---|
  | 10 | 96 | 106 | 236 | 2.32 % | 6.29 % |
  | **15** | **141** | **156** | **351** | **1.60 %** | **4.30 %** |
  | 25 | 231 | 256 | 581 | 0.98 % | 2.63 % |
  | 50 | 456 | 506 | 1156 | 0.50 % | 1.34 % |

  L'osservazione da fare è che **la sparsità è un vantaggio che cresce con N**: a N = 15 la
  Jacobiana è densa all'1.6 %, a N = 50 allo 0.5 %. Su un orizzonte corto come quello deployato
  il guadagno strutturale del multiple shooting è quindi il più piccolo dell'intervallo, e il
  confronto con il single shooting va misurato invece che assunto — potrebbe non vincere.
- **Verifica** — la tabella sopra estesa al single shooting; iterazioni e tempo per solve delle
  due formulazioni sullo stesso ciclo estratto dalle bag.
- **Sforzo** — **M** · `guides/snippets/nlp_structure.py` produce già metà della tabella

---

### 1.6 Parametrizzazione dell'ingresso e move blocking

- **Corso** — §7.2.3: feedback di stato parametrizzato eq. (3.13), **move blocking** (ingresso
  costante su più intervalli), strategie di commutazione per ingressi discreti. Motivazione:
  disaccoppiare il numero di variabili dall'orizzonte.
- **Stato** — **assente**: 45 ingressi liberi su N = 15. C'è però già una penalità sul jerk
  (`R_jerk = 0.4`), che è una regolarizzazione degli incrementi ma non una riduzione delle
  variabili.
- **Da aggiungere** —
  1. **move blocking** con blocchi crescenti (1,1,1,2,2,4,4): l'orizzonte resta 3 s ma le
     variabili scendono da 45 a ~21. Su un problema che gira a 8 Hz nominali questo è
     direttamente budget di calcolo recuperato — e quel budget serve, vista la §1.3;
  2. **forma in incrementi**: ottimizzare Δu con `u_k = u_{k−1} + Δu_k` e `u_{−1}` nello stato.
     Rende naturali i vincoli di rate e migliora il condizionamento. Nota: data la
     degenerazione del §0.2, lo stato velocità **è già** `u_{k−1}`, quindi questa
     riformulazione qui costa zero stati aggiuntivi. È un allineamento fra teoria e
     implementazione che vale la pena rivendicare.
- **Verifica** — Pareto variabili/tempo di solve contro qualità della traiettoria; da fare
  insieme allo sweep della §1.3.
- **Sforzo** — **S/M**

---

### 1.7 Omotopia e warm starting

- **Corso** — §7.1.1 eq. (7.2), Fig. 7.1: risolvere una successione di problemi con un peso che
  decresce gradualmente, inizializzando ogni solve con la soluzione precedente. §7.2.5 ultimo
  punto: warm start in NMPC con la coda della sequenza precedente.
- **Stato** — warm start **sì** (soluzione shiftata di un passo, con svuotamento su fallimento
  e su picco di costo), omotopia **no**, nessun confronto con un baseline.
- **Da aggiungere** —
  1. **Baseline del warm start**: confrontare tre inizializzazioni — soluzione shiftata,
     traiettoria di riferimento, cold start a zero — su iterazioni medie e tempo. Il
     riferimento viene calcolato comunque a ogni ciclo, quindi è il baseline onesto, non lo
     zero. Ora che `iter_count` è esposto, la misura è diretta.
     *(Nel progetto gemello questo esperimento ha dato un risultato controintuitivo: la
     soluzione shiftata perde contro il riferimento in 4 casi su 4.)*
  2. **Omotopia su α**: partire con `obs_alpha` piccolo (barriera morbida, problema ben
     condizionato), risolvere, alzare α e riusare la soluzione, ripetere. È la continuazione
     della §2.5 e la ricetta della §7.1.1 applicata alla lettera.
- **Verifica** — iterazioni e tasso di successo, per α fisso vs α in omotopia, sulle bag
  esistenti. `decision_plane.py --set` fa già gli sweep parametrici.
- **Sforzo** — **S** (baseline) + **M** (omotopia)

---

### 1.8 Ottimizzazione multi-obiettivo fatta come da programma

- **Corso** — §7.4: tre strategie (vincoli su tutti gli obiettivi tranne uno eq. (7.8); somma
  pesata eq. (7.9); mista). Procedura a-posteriori: **(I)** normalizzare gli obiettivi allo
  stesso ordine di grandezza, **(II)** risolvere ripetutamente campionando i pesi sul simplesso
  `A = {α ≥ 0, Σαᵢ = 1}` includendo i vertici, **(III)** post-processare. Definizioni: fronte
  di Pareto, punti non dominati, **punto Utopico**, scelta come punto più vicino all'Utopico in
  norma 2. Strumenti grafici: **curva di Pareto** (Fig. 7.9) e **spider chart** (Fig. 7.10).
  Avvertenza: la somma pesata recupera il fronte completo solo se questo è convesso.
- **Stato** — **presente ma metodologicamente incompleto**. Il punteggio composito in
  [`tuning/`](../tuning/) è una somma pesata di 5 obiettivi (successo, efficienza di percorso,
  smoothness, clearance, tempo) con pesi fissi scelti a mano: è la strategia 2 del §7.4,
  valutata **in un solo punto α**. Mancano normalizzazione dichiarata, campionamento del
  simplesso, fronte, punto Utopico e grafici.
- **Da aggiungere** — cambiare procedura, non strumenti:
  1. normalizzare i 5 obiettivi (punto I);
  2. campionare α sul simplesso, includendo i 5 vertici — che danno il punto Utopico (punto II);
  3. tracciare la curva di Pareto sulle coppie più significative (lunghezza percorso vs tempo
     al goal; clearance minima vs tempo), lo spider chart di 2-3 soluzioni non dominate, e la
     scelta finale come punto più vicino all'Utopico;
  4. dichiarare esplicitamente se il fronte osservato è convesso, e se non lo è, dirlo.
- **Perché conta** — è il modo di trasformare la parte più corposa e più fuori programma del
  repository in contenuto d'esame (§8).
- **Sinergia** — la §1.1 introduce naturalmente tre obiettivi (accuratezza, sforzo, tempo di
  completamento) con i pesi α₁, α₂, α₃ già nella forma della eq. (7.9): il fronte di Pareto su
  quei tre è il caso di studio più pulito, e viene gratis.
- **Sforzo** — **M**

---

### 1.9 Evitare la programmazione mista-intera: rivendicarlo

- **Corso** — §4.2.6: i MIP non sono né lisci né convessi, l'enumerazione esplode, *"è
  consigliabile evitare un gran numero di variabili intere"*; una parametrizzazione adeguata
  può mappare variabili continue su decisioni discrete.
- **Stato** — la scelta c'è, l'argomento no.
- **Da aggiungere** — un paragrafo: la decisione discreta "da che lato aggiro l'ostacolo" è
  intrinsecamente combinatoria e in una formulazione monolitica richiederebbe variabili
  binarie. Qui viene delegata ad A\* sulla griglia di occupazione, e l'NLP resta **continuo e
  liscio**. È il motivo architetturale per cui esiste la separazione A\* / MPC, oggi presentata
  come dettaglio implementativo.

  Il prezzo va detto: l'ottimalità è quella del percorso scelto da A\*, non globale. E il dato
  del §0.3 lo rende quantitativo — ai pesi deployati la scelta di omotopia la fa **interamente**
  A\*, perché l'NLP non ha nemmeno due minimi fra cui scegliere.
- **Sforzo** — **S**

---

### 1.10 MPC approssimato per approssimazione di funzione

- **Corso** — §7.2.5: se l'FHOCP non è convesso la legge di controllo può essere discontinua;
  diversi approcci usano reti neurali per derivare offline leggi NMPC approssimate da un
  insieme di punti `u⁽ⁱ⁾ = κ_MPC(z⁽ⁱ⁾)`, riducendo il carico online.
- **Stato** — [`PointCloud-GNNencoder/`](../src/PointCloud-GNNencoder/) esiste ed è di fatto
  fuori programma. **Questo è il suo unico aggancio legittimo.**
- **Raccomandazione** — **escluderlo** (§8). Riformularlo richiederebbe di sviluppare davvero
  l'approssimazione della legge di controllo, che è un progetto a sé; citarlo senza svilupparlo
  non aggiunge contenuto.
- **Sforzo** — **S** (esclusione) / **L** (svilupparlo)

---

## 2. Capitolo 6 — Ottimizzazione vincolata *(il macchinario)*

### 2.1 KKT, LICQ e moltiplicatori per *questo* problema

- **Corso** — §6.1.1 (cono tangente, active set, LICQ Def. 6.1.5), §6.1.2 (lagrangiana
  eq. (6.7), lemma di Farkas Thm 6.1.3, condizioni KKT eq. (6.8) con complementarità), §6.1.3
  (cono delle direzioni critiche eq. (6.11), NOC-C-2 Thm 6.1.5, SOC-C-2 Thm 6.1.6, Hessiana
  proiettata).
- **Stato** — **assente**.
- **Da aggiungere** — la trattazione completa, che qui è **corta e pulita** perché i vincoli
  sono semplici:

  ```
  L(X, U, λ, μ) = J(X, U)
                − Σ_k λ_kᵀ [X_{k+1} − f(X_k, U_k)]    − λ_0ᵀ [X_0 − x̂]
                − Σ_k μ_kᵀ h_box(U_k)
  ```

  - **LICQ vale banalmente**: i vincoli attivi sono box su componenti distinte di U, i cui
    gradienti sono versori distinti, quindi linearmente indipendenti (Def. 6.1.5). Si dimostra
    in tre righe e si può dichiarare senza cautele.
  - **Moltiplicatori**: leggerli con `opti.dual(constraint)` e mostrare quali box sono attivi
    lungo l'orizzonte. Sul G1 l'aspettativa è specifica e verificabile: `vy_max = 0.02` m/s è
    così stretto che il box laterale dovrebbe essere **quasi sempre attivo**, mentre
    `U[0] ≥ 0` (nessuna retromarcia) dovrebbe attivarsi nelle manovre strette.
  - **Complementarità stretta**: verificare se ci sono vincoli debolmente attivi (h_i = 0 e
    μ_i = 0), il caso in cui il cono critico non degenera in un sottospazio (§6.1.3).
  - **SOC-C-2**: proiettare l'Hessiana della lagrangiana sul cono critico e verificarne la
    definita positività ⇒ certificato di minimo locale, il massimo ottenibile in un problema
    non convesso.
- **FATTO** — [`viz/kkt_analysis.py`](../viz/kkt_analysis.py), che opera su un ciclo reale
  estratto da una bag. Risultati su `industrial_plant_fix`, nove cicli lungo la missione:

  | ciclo | vincoli attivi | rango | LICQ | dim. cono critico | compl. stretta | λ_min proiettato |
  |---|---|---|---|---|---|---|
  | 50 | 96 | 96 | ✓ | 45 | ✓ | +8.78e-01 |
  | 250 | 101 | 101 | ✓ | 40 | ✓ | +8.84e-01 |
  | 450 | 135 | 135 | ✓ | 6 | ✓ | +3.71e+00 |
  | 650 | 138 | 138 | ✓ | 3 | ✓ | +2.16e+01 |
  | 718 | 140 | 140 | ✓ | 1 | ✓ | +7.83e+02 |

  - **LICQ vale in ogni ciclo** (rango = numero di vincoli attivi) ⇒ i moltiplicatori KKT
    esistono e sono unici;
  - **complementarità stretta in ogni ciclo** ⇒ il cono critico coincide con il nucleo del
    Jacobiano attivo, e SOC-C-2 si verifica esattamente invece che per difetto;
  - **SOC-C-2 soddisfatta ovunque** (λ_min > 0) ⇒ certificato di minimo locale stretto, il
    massimo ottenibile in un problema non convesso;
  - stazionarietà: `‖∇_x L(x*, λ*)‖_∞ = 6.1e-11` contro `‖∇_x f(x*)‖_∞ = 1.6e+04`.

- **Il risultato che vale la pena raccontare** — la **dimensione del cono critico collassa
  da 45 a 1** lungo la missione. A fine corsa restano 141 variabili e 140 vincoli attivi:
  un solo grado di libertà. L'MPC passa da *guidato dal costo* a **guidato dai vincoli**, e
  nella fase finale non sta più scegliendo una traiettoria, la sta subendo.

  La ripartizione dei vincoli attivi lo spiega: `|vy| ≤ vy_max` e `|w| ≤ omega_max` sono
  saturati **15 volte su 15** nei cicli finali, e `vx ≥ 0` 14 su 15. Conferma quantitativa
  della previsione fatta qui sopra: `vy_max = 0.02` m/s è così stretto da essere sempre
  attivo, e di fatto rimuove un grado di libertà invece di limitarlo.

- **Due trappole trovate implementandolo**, che vale la pena documentare perché ricorrono:
  1. Opti canonizza ogni vincolo come `lbg ≤ g(x) ≤ ubg` e **assorbe nei limiti** il membro
     destro quando è un parametro: `X[:,0] == p_x0` diventa `g = X[:,0]`, `lbg = ubg = p_x0`.
     Valutare `|g|` su quelle righe restituisce *lo stato*, non il residuo. Il residuo è
     sempre `g − lbg`.
  2. Due righe distinte possono condividere la stessa espressione con limiti diversi
     (`U[0,k] ≥ 0` e `U[0,k] ≤ vx_max` sono entrambe la riga `U[0,k]`). L'attivazione va
     quindi decisa sulla **distanza dal limite**, non su `|g| ≈ 0`, altrimenti si contano
     attivi entrambi i lati di uno stesso box.
- **Sforzo** — **M** · **priorità massima**: sblocca la §2.2 e la §2.3

---

### 2.2 Vincolo ostacolo *vero* con slack, e penalità esatta ℓ¹ ⭐

- **Corso** — §6.3.3 eq. (6.28) merit function ℓ¹, **Thm 6.3.1**: se `σ_i > |λ_i*|` e
  `τ_i > |μ_i*|`, allora x\* è minimizzatore anche della funzione di merito non vincolata. La
  penalità ℓ¹ è **esatta**: lo slack va a zero non appena ρ supera il moltiplicatore. Una
  penalità quadratica lascia invece un residuo `s* ≈ μ*/(2ρ)` per ogni ρ finito.
- **Stato** — **assente e sostituito da un'euristica**. Oggi gli ostacoli sono solo un costo,
  senza slack e senza vincolo; il peso `W_obs_sigmoid` è un iperparametro tarato per tentativi.
- **Perché è il buco più grande** — il §0.3 lo quantifica: ai pesi deployati il termine
  ostacolo non riesce nemmeno a creare un secondo minimo locale. Non è che l'MPC scelga male
  come evitare l'ostacolo; è che non sceglie affatto.
- **Da aggiungere** —

  ```
  vincolo:  ‖p_k − o_j‖ ≥ d_safe − s_{jk},   s_{jk} ≥ 0
  costo:    + ρ Σ_{j,k} s_{jk}        (ℓ¹, esatta, lineare ⇒ NLP resta liscio)
       o    + ρ Σ_{j,k} s_{jk}²       (ℓ², residuo non nullo)
  ```

  Procedura: risolvere una volta con vincolo hard, leggere μ\* dalla soluzione duale (§2.1),
  scegliere ρ > max|μ\*|. **A quel punto ρ smette di essere un parametro da tarare** e diventa
  una quantità letta dal problema.

  L'infrastruttura per l'ultimo passo esiste già: `decision_plane.py` calcola σ = 1.5·max|λ|
  dai moltiplicatori veri di IPOPT (misurato: σ = 3.996·10⁴ sul ciclo 718 di
  `industrial_plant_fix`).
- **FATTO** — `MPCConfig.obstacle_mode` ∈ {`penalty`, `l1`, `l2`} in `MPCTracker`, più
  [`viz/exact_penalty.py`](../viz/exact_penalty.py) per lo sweep. Il modo deployato resta
  `penalty`: verificato che `J*` sia **bit-identico** al valore pre-modifica
  (8177.231314839336) e che i vincoli restino 156, cioè nessuna variabile aggiunta.

  Scenario `narrow_gap`, `d_safe = 1.1` m (scelto perché il vincolo **morda**: la clearance
  naturale è 0.76 m). Moltiplicatore letto dal problema: **max|μ\*| = 8.905e+03**.

  | ρ | max s\* (ℓ¹) | max s\* (ℓ²) |
  |---|---|---|
  | 1e+03 | 1.457e-01 | 2.606e-01 |
  | **1e+04** | **0** (−9.8e-09) | 1.297e-01 |
  | 1e+05 | **0** | 3.975e-02 |
  | 1e+06 | **0** | 4.375e-03 |
  | 1e+07 | **0** | 4.420e-04 |
  | 1e+08 | **0** | 4.424e-05 |

  - **ℓ¹**: lo slack diventa **esattamente nullo** a partire da ρ = 1e4 — e la soglia teorica
    max|μ\*| = 8.9e3 cade esattamente fra 1e3 (dove s\* = 0.146) e 1e4. **Thm 6.3.1
    confermato sperimentalmente.**
  - **ℓ²**: il residuo decresce con pendenza log-log **−1.00 esatta** sulla coda (−0.78 se si
    include il regime pre-asintotico) e **non si annulla mai**: 4.4e-05 ancora a ρ = 1e8.

  ρ smette quindi di essere un iperparametro: si legge dal problema.

- **Due difetti di formulazione emersi implementandolo**, entrambi di interesse teorico:
  1. **Il vincolo non va imposto a k = 0.** Lì lo stato è fissato da `X[:,0] == x0`, quindi il
     vincolo non dipende da alcuna variabile decisionale: imporlo rende l'NLP inammissibile
     ogni volta che il robot si trova già entro `d_safe` da un ostacolo — cioè proprio quando
     servirebbe. Misurato: con `d_safe = 0.9` m sulla bag, lo slack saturava a 0.1999 =
     esattamente la violazione a k = 0 (clearance corrente 0.7017 m). Ora il vincolo parte da
     k = 1 e lo slack ha N colonne, non N+1.
  2. **L'inammissibilità residua è colpa di U_Σ, non dei dati.** Anche partendo da k = 1, sul
     ciclo reale `d_safe = 0.75` m resta inammissibile: con `vx ≥ 0` e `vy_max = 0.02` m/s il
     robot in un passo può solo avanzare di 0.06 m lungo la propria direzione, e non può
     arretrare lateralmente dal muro di punti LiDAR. È l'interazione fra l'asimmetria
     dell'insieme ammissibile degli ingressi (§5.1) e un vincolo di stato, e collega
     direttamente questa scheda alla §1.2 (fattibilità ricorsiva).
- **Impatto sul repo** — rende superflua una parte della campagna BO, il pezzo più fuori
  programma del progetto (§8).
- **Costo da mettere in conto** — la formulazione vincolata porta le disuguaglianze da 60 a
  oltre 300 e le iterazioni da ~20 a 60-240: il cap deployato `mpc_max_iter = 40` non basta.
  Questo **ribalta l'argomento della §2.4** (active-set contro interior-point), come previsto.
- **Sforzo** — **M** · dipende da §2.1

---

### 2.3 SQP e Gauss-Newton vincolato scritti a mano

- **Corso** — §6.3.1 (SQP con Hessiana esatta, eq. (6.22)), §6.3.2 (**Gauss-Newton vincolato**,
  eq. (6.24)-(6.26); il corso lo definisce *"spesso il metodo migliore nei problemi di stima e
  controllo"*; e BFGS vincolato con trucco di Powell), §6.3.3 (line search con merit function),
  §6.3.4 (**Algoritmo 6.3.1** completo).
- **Stato** — **assente**. È il cuore dei laboratori del corso, quello che si scrive a mano.
- **Nota favorevole** — il costo di questo problema è **quasi** una somma di quadrati: tracking
  `‖e‖²_Q`, sforzo `‖u‖²_R`, jerk `‖Δu‖²`, penetrazione `max(0, ·)²` lo sono; **solo la
  sigmoide non lo è**. Quindi F(x) si costruisce direttamente e `H = 2 ∇F ∇Fᵀ` è disponibile
  senza derivate seconde, con `H ⪰ 0` garantito — che è precisamente l'argomento con cui si
  sceglie GN su un problema non convesso: evita QP interni non convessi.
- **Da aggiungere** — implementare l'Algoritmo 6.3.1 con:
  - QP interno risolto con `qpOASES` o `osqp`;
  - Hessiana da Gauss-Newton vincolato, e in alternativa BFGS + Powell;
  - back-tracking con merit function ℓ¹ e condizione di Armijo eq. (6.36);
  - aggiornamento dei pesi σ, τ come al punto 5 dell'Algoritmo 6.3.1.

  Due varianti dell'obiettivo, per rendere GN applicabile: (a) sostituire la sigmoide con la
  sola hinge² (somma di quadrati pura), (b) tenerla e trattarla come termine extra
  nell'Hessiana approssimata.
- **Verifica** — confronto con IPOPT sullo stesso ciclo estratto dalle bag: iterazioni, tempo
  per solve, costo finale, traiettoria. Attesa dalla teoria: GN lineare lontano dalla soluzione,
  quadratico vicino; IPOPT più robusto ma con più lavoro per iterazione. Il riferimento
  numerico c'è già: media 13.4 iterazioni, 70 ms per solve.
- **Sforzo** — **L** · il pezzo più corposo, e il più caratterizzante per l'esame

---

### 2.4 Active-set contro interior-point

- **Corso** — §6.2.1 (QP con sole uguaglianze: sistema KKT eq. (6.15), risolubile
  direttamente), §6.2.2 (active-set Fig. 6.10 vs interior-point, barriera logaritmica
  eq. (6.18), central path, τ → 0, Fig. 6.11–6.12). La regola pratica del corso: **active-set
  conviene con poche disuguaglianze, interior-point con molte**.
- **Stato** — IPOPT usato senza alcuna motivazione scritta.
- **Da aggiungere** — l'osservazione che oggi le disuguaglianze sono **60 righe, tutte box
  sugli ingressi**, cioè esattamente la situazione in cui il corso dice che l'active-set è
  competitivo. Provare `qpOASES` dentro SQP (§2.3) e confrontare.

  **Il punto interessante è che l'argomento si ribalta**: dopo la §2.2 gli ostacoli diventano
  vincoli e le disuguaglianze passano da 60 a 60 + 128·2, cioè da poche a molte. Il confronto
  va quindi fatto **prima e dopo**, ed è un risultato di per sé: la scelta del solutore QP non
  è una preferenza, è una conseguenza della formulazione.
- **Sforzo** — **M** · dipende da §2.3

---

### 2.5 La barriera dell'interior-point e la barriera degli ostacoli sono la stessa cosa

- **Corso** — §6.2.2 eq. (6.18): l'interior-point sostituisce le complementarità non lisce con
  `−τ Σ ln(C_i x + d_i)`, τ decrescente. Fig. 6.12: al calare di τ la barriera diventa più
  ripida e il condizionamento peggiora.
- **Da aggiungere** — il parallelo esplicito: `obs_alpha` nella sigmoide gioca **esattamente**
  il ruolo di 1/τ. Barriera più ripida = evitamento migliore e condizionamento peggiore, la
  stessa identica curva di compromesso. Trasforma un iperparametro tarato per tentativi in un
  oggetto teorico noto, ed è l'aggancio naturale alla §1.7 (omotopia).
- **Verifica** — sweep di `obs_alpha` con iterazioni IPOPT e numero di condizionamento
  dell'Hessiana. Misura diretta ora che `iter_count` è esposto.
- **Sforzo** — **S**

---

## 3. Capitolo 2 — Modelli e discretizzazione

### 3.1 Ordine di troncamento: Euler contro mid-point (RK2)

- **Corso** — §2.1.3, eq. (2.9) Euler in avanti, eq. (2.10) RK2 / regola del punto medio,
  Fig. 2.3–2.4. Euler ha errore locale O(dt²) e globale O(dt); RK2 locale O(dt³), globale O(dt²).
- **Stato** — Euler puro sul canale di posizione: `cos(yaw_k)`, `sin(yaw_k)` valutati a
  **inizio** intervallo, con disallineamento sistematico ω·dt/2. Sul profilo G1
  (dt = 0.2 s, ω_max = 0.3 rad/s) sono **0.03 rad per passo**, cioè 1.7°, che su 15 passi si
  accumulano.
- **Da aggiungere** — regola del punto medio sul solo canale di posizione:

  ```python
  yaw_eval = yaw_k + 0.5 * wz_next * dt      # RK2, eq. (2.10)
  px_next  = px_k + (vx_next*cos(yaw_eval) - vy_next*sin(yaw_eval)) * dt
  py_next  = py_k + (vx_next*sin(yaw_eval) + vy_next*cos(yaw_eval)) * dt
  ```

  Costo computazionale: una addizione (`wz_next` serve comunque).
- **FATTO** — `MPCConfig.integrator` ∈ {`euler`, `midpoint`}, esposto come parametro ROS
  `mpc_integrator` e impostato a `midpoint` nel profilo G1. Verificato da
  [`tests/test_integrators.py`](../tests/test_integrators.py) contro la soluzione esatta in
  forma chiusa (per ω ≠ 0 lo spostamento è un arco:
  `Δ_world = (1/ω) R(ψ₀) [[sin a, cos a − 1], [1 − cos a, sin a]] v`, `a = ω dt`).

  | dt [s] | errore Euler [m] | errore punto medio [m] |
  |---|---|---|
  | 0.200 | 1.740e-02 | 8.700e-05 |
  | 0.100 | 8.699e-03 | 2.175e-05 |
  | 0.050 | 4.350e-03 | 5.437e-06 |
  | 0.025 | 2.175e-03 | 1.359e-06 |
  | 0.0125 | 1.087e-03 | 3.398e-07 |

  **Ordine stimato: 1.00 per Euler, 2.00 per il punto medio**, identico su tre regimi
  (nominale, con deriva laterale, rotazione rapida). Il test verifica anche che la dinamica
  costruita dentro l'NLP coincida con lo schema di riferimento: scarto **0.000e+00**.

- **Il risultato onesto in anello chiuso** — a orizzonte recedente il guadagno **quasi
  sparisce**: su quattro scenari, il costo mediano migliora dello 0.0-1.1 % e distanza finale
  e clearance sono invariate. La ragione è strutturale: si applica solo il **primo** ingresso
  e A\* ripianifica, quindi l'errore di predizione accumulato sui 15 passi non si trasferisce
  alla traiettoria percorsa. *(Il confronto è pulito: l'impianto simulato usa un
  aggiornamento di Euler fisso, indipendente da `cfg.integrator`; cambia solo il modello di
  predizione.)*

  Il valore sta quindi nella **fedeltà della predizione**, non nella traiettoria: conta per
  la §1.3 (orizzonti più lunghi, dove l'errore si accumula su più passi) e per la §1.4 (dove
  l'errore di predizione va attribuito al disallineamento di modello, non all'integratore).
- **Perché conta più qui che sul Go2** — dt è raddoppiato (0.2 contro 0.1 s), e l'errore di
  Euler è lineare in dt: l'errore per passo è raddoppiato rispetto al profilo originale.
- **File** — `mpc_tracker.py::_build_nlp`, nuovo `tests/test_integrators.py`
- **Sforzo** — **S** · miglior rapporto risultato/sforzo di tutta la guida

---

### 3.2 Scelta del passo di campionamento

- **Corso** — §2.1.3. La discretizzazione è esatta agli istanti di campionamento solo se la
  banda del segnale è rispettata; sotto-campionare produce aliasing e distorsione spettrale, e
  il modello di predizione smette di rappresentare l'impianto.
- **Stato** — **assente**. `dt = 0.2` s ed `mpc_rate_hz = 8.0` sono ereditati dalla taratura
  hardware, senza analisi.
- **Osservazione misurata** — il ciclo gira in realtà a **2.6 Hz** (775 cicli in 298.6 s), non
  a 8. Il periodo effettivo è ~0.38 s, quasi **due passi di orizzonte**: il modello di
  predizione avanza a dt = 0.2 s ma il feedback arriva ogni 0.38 s. È un disallineamento fra
  modello e implementazione che va o corretto o dichiarato.
- **Da aggiungere** —
  1. l'analisi di banda del modello: con il lag degenere (§0.2) il canale velocità non ha
     dinamica propria, quindi la banda rilevante è quella della **cinematica**, governata da
     ω_max e dalla curvatura del percorso;
  2. lo studio dell'effetto di dt sull'errore di predizione a orizzonte fisso (3 s), separando
     l'errore di discretizzazione (§3.1) da quello di modello (§1.4);
  3. **la diagnosi del rate effettivo**: perché 2.6 Hz e non 8. Va guardato, perché indebolisce
     il feedback e non è una scelta di progetto.
- **Verifica** — errore di predizione vs dt a orizzonte temporale costante; istogramma del
  periodo effettivo di ciclo dalle bag.
- **Sforzo** — **S** (analisi) + **M** (diagnosi del rate)

---

### 3.3 Discretizzazione esatta ZOH: quando è disponibile, e quando è vuota

- **Corso** — §2.1.3: quando l'ingresso è costante a tratti (ZOH), la traiettoria campionata si
  può calcolare esattamente invece che approssimarla. È l'esempio didattico di "scegliere la
  discretizzazione in base alla struttura del modello".
- **Stato** — il codice usa `lag = 1 − exp(−dt/τ)`, che **è** la ZOH esatta di un primo ordine.
  Sul profilo Go2 (τ = 0.12 s, dt = 0.1 s) il coefficiente vale 0.565 ed è un punto di forza
  reale mai rivendicato.
- **Ma sul G1 è vuota** — con τ = 0.001 s e dt = 0.2 s il coefficiente vale **1.000000000000**
  (§0.2). La formula esatta restituisce il caso degenere `v_{k+1} = u_k`: formalmente corretta,
  fisicamente priva di contenuto.
- **Da aggiungere** — mezza pagina che dice **entrambe** le cose:
  1. il sottosistema velocità è LTI del primo ordine con ingresso costante a tratti per
     costruzione, quindi la ZOH esatta è disponibile e non c'è ragione di usare Euler lì;
  2. ai parametri deployati sul G1 quella struttura **collassa**, e il modello effettivo è un
     uniciclo a 3 stati con un passo di ritardo. Le tre variabili di velocità nello stato sono
     ridondanti: si potrebbero eliminare, riducendo le variabili decisionali da 141 a ~96.

  Riconoscerlo è più forte che rivendicare un modello che non è quello in esecuzione — e apre
  due strade concrete: eliminare la ridondanza (meno variabili, §1.5), oppure identificare un
  τ realistico per il controllore di camminata del G1 e recuperare la dinamica.
- **Sforzo** — **S**

---

## 4. Capitolo 5 — Calcolo delle derivate

### 4.1 AD contro differenze finite: dichiararlo e misurarlo

- **Corso** — §5.2 (differenze finite in avanti e centrate; passo ottimo `√eps ≈ 10⁻⁸` e
  `eps^{1/3} ≈ 7.6·10⁻⁶`; accuratezza attesa ≈ 10⁻⁸ e ≈ 10⁻¹¹; costo n+1 e 2n valutazioni),
  §5.3 (AD forward < 2n, AD backward **< 3 valutazioni indipendentemente da n**, con CasADi
  citato esplicitamente come riferimento [2]), §5.4 (trucco della parte immaginaria).
- **Stato** — CasADi fa AD e nel repo non c'è una riga che lo dica.
- **Da aggiungere** —
  1. sezione di mezza pagina: con n = 141 variabili, le differenze finite centrate costerebbero
     282 valutazioni per gradiente a precisione ≈ 10⁻¹¹, contro < 3 dell'AD backward a
     precisione macchina. A 8 Hz nominali è la differenza fra fattibile e non fattibile — e il
     rapporto peggiora linearmente se si allunga N (§1.3), quindi è **l'argomento che rende
     discutibile un orizzonte più lungo**;
  2. misura empirica: risolvere lo stesso NLP con Hessiana approssimata
     (`hessian_approximation: limited-memory`, cioè L-BFGS) contro l'Hessiana esatta da AD, e
     riportare iterazioni e tempo. È il confronto §4.4.4 (Newton esatto vs quasi-Newton)
     ottenuto senza scrivere un solutore.
- **Verifica** — tabella `t_proc_nlp_f`, `t_proc_nlp_grad_f`, `t_proc_nlp_jac_g`,
  `t_proc_nlp_hess_l` con la quota di tempo nelle derivate. CasADi le restituisce già in
  `sol.stats()`; oggi ne leggiamo solo `iter_count`.
- **Sforzo** — **S** · alto rendimento, quasi tutto già disponibile

---

### 4.2 Ricorsione delle sensitività come controllo incrociato

- **Corso** — §5.1 eq. (5.3): propagazione di `∇_θ ẑ(i, θ)` insieme alla traiettoria.
- **Stato** — assente.
- **Da aggiungere** — con l'identificazione fuori perimetro, questa voce resta utile in un ruolo
  diverso: **verifica di correttezza dell'AD**. Propagare a mano la sensitività della
  traiettoria predetta rispetto a x₀ lungo i 15 passi, e confrontarla con
  `ca.jacobian(X, p_x0)` calcolata da CasADi. Se coincidono a precisione macchina, si è
  dimostrato di aver capito cosa fa l'AD invece di averlo solo usato.

  È anche il modo di ottenere ∂x\*/∂ϑ per la §5.4 (regolarità, Thm 4.4.6).
- **Sforzo** — **S**

---

## 5. Capitolo 4 — Fondamenti e problemi non vincolati

### 5.1 Classificare il problema

- **Corso** — §4.1 (forma standard, Thm 4.1.1 esistenza del minimo globale con Ω compatto),
  §4.2 (LP / QP / convesso / SDP / MIP), §4.2.5 (differenziabile vs non liscio).
- **Stato** — **assente**: non è mai scritto *che tipo* di programma sia.
- **Da aggiungere** — una sezione breve ma precisa:
  - è un **NLP non convesso**, non un QP: la dinamica è nonlineare (R(ψ) moltiplica u) e i
    termini ostacolo sono non convessi;
  - `U_Σ` è compatto (box), `X_Σ` no (gli stati non hanno limiti superiori) ⇒ il Thm 4.1.1 non
    si applica direttamente; l'esistenza si recupera osservando che il costo è radialmente
    illimitato negli stati penalizzati;
  - il termine di penetrazione `max(0, r − d)²` è **C¹ ma non C²**: la sua Hessiana salta sulla
    superficie di attivazione `d = r`, e IPOPT è un metodo di tipo Newton che usa derivate
    seconde (§4.2.5, §4.4.4). La sigmoide invece è C^∞;
  - i box sull'ingresso sono **asimmetrici**: `U[0] ≥ 0` vieta la retromarcia. Va detto, perché
    restringe l'insieme ammissibile in modo qualitativamente diverso da un box simmetrico e
    interagisce con la §1.2 (l'equilibrio `v = 0` è sul bordo, non all'interno).
- **Sforzo** — **S**

---

### 5.2 Analisi di convessità dei due termini ostacolo

- **Corso** — §4.3.1 (condizioni del primo e secondo ordine, Thm 4.3.1–4.3.2), §4.3.2
  (operazioni che preservano la convessità), §4.3.3 (Thm 4.3.3: nei problemi convessi ogni
  minimo locale è globale) e l'esempio finale di riscrittura convessa con cambio di variabile.
- **Stato** — **assente**, ed è un punto in cui è facile sbagliare.
- **Da aggiungere** — l'analisi corretta, che dà un risultato **negativo su entrambi i termini**:
  - **sigmoide** `0.5(1 − tanh(0.5 α (d − r)))`: non convessa per costruzione (convessa oltre
    il flesso, concava prima);
  - **penetrazione** `max(0, r − ‖p − o‖)²`: **anch'essa non convessa**. `‖p − o‖` è convessa,
    quindi `r − ‖p − o‖` è **concava**, e la composizione di una funzione convessa decrescente
    con una funzione concava non è convessa (§4.3.1: la regola di composizione richiede la
    monotonia *crescente*). Attenzione: **non** è lo stesso caso della hinge su un semipiano
    `max(0, d_safe − nᵀ(p − o))²`, che essendo hinge di una funzione **affine** è convessa.
- **Il collegamento con la §3.3 delle dispense (convessificazione)** — la Successive Convex
  Approximation del vincolo di collisione: linearizzare `‖p − o‖ ≥ d_safe` attorno a una
  traiettoria di riferimento dà il semipiano **affine** `nᵀ(p − o) ≥ d_safe`, che è convesso.
  Confrontare le due formulazioni a parità di scenario. *(Già implementata e testata nel
  progetto gemello: `trajopt_core/mpc/obstacles.py::SlackedHalfSpace`.)*
- **Verifica** — autovalori dell'Hessiana del termine ostacolo su una griglia attorno a un
  ostacolo, mostrando dove diventa indefinita. `cost_field.py` costruisce già il campo.
- **Sforzo** — **M**

---

### 5.3 Metodi di Newton, tassi di convergenza, line search

- **Corso** — §4.4.2 (line search, Armijo eq. (4.19), Alg. 4.4.1–4.4.2, Thm 4.4.5 convergenza
  globale, trust region), §4.4.3 (Q-lineare / R-lineare / Q-quadratico / Q-superlineare, e il
  legame `c = (λ_max − λ_min)/(λ_max + λ_min)` con il condizionamento), §4.4.4 (Newton esatto,
  Gauss-Newton, gradient descent, BFGS con trucco di Powell).
- **Stato** — **parzialmente sbloccato**: `iter_count` è ora esposto (media 13.4, massimo 29 su
  una missione reale). Il resto delle statistiche no.
- **Da aggiungere** —
  1. esporre anche `return_status` e i `t_proc_nlp_*` (§4.1);
  2. **studio del condizionamento**: la ripidità della barriera `obs_alpha` gioca il ruolo del
     parametro di barriera inverso; al crescere di α il numero di iterazioni e il numero di
     condizionamento dell'Hessiana peggiorano. È il legame diretto tra §4.4.3 e §6.2.2 e si
     misura con uno sweep (§2.5).
- **Verifica** — grafico iterazioni medie e p95 vs α; quota di tempo nelle callback derivative.
- **Sforzo** — **S** (logging) + **M** (sweep)

---

### 5.4 Regolarità della soluzione rispetto ai parametri

- **Corso** — §4.4.5, Thm 4.4.6: se in x\*(ϑ̄) valgono le SOC-2, allora x\*(ϑ) è ben definita e
  differenziabile in un intorno, con `∂x*/∂ϑ = −(∇²_x f)⁻¹ ∂∇_x f/∂ϑ`. Fig. 4.17: esempio di
  biforcazione, dove questa regolarità si perde.
- **Stato** — **assente come teoria, presente come sintomo**. In `mpc_tracker.py`:
  `_COST_SPIKE_FACTOR = 5.0` con svuotamento del warm start quando il costo supera 5× la media
  recente. È una toppa empirica esattamente al fenomeno del Thm 4.4.6.
- **Risultato già misurato, e va usato** — la biforcazione **non c'è** ai pesi deployati
  (§0.3): due warm start opposti danno soluzioni a distanza 0.0000 in ℝ¹⁴¹, su due missioni
  reali indipendenti. Compare fra W_obs 300 e 600. Quindi:
  - `_COST_SPIKE_FACTOR` sta proteggendo da un fenomeno che, ai pesi attuali, **non si verifica**;
  - la soglia di biforcazione misurata è il modo pulito di presentare la Fig. 4.17 su dati
    propri: uno sweep di W_obs con la distanza fra le due soluzioni in ordinata, che passa da
    zero a non-zero.
- **Da aggiungere** — completare lo sweep W_obs ∈ {120, 200, 300, 450, 600, 900} tracciando
  distanza fra soluzioni da warm start opposti, e sovrapporre il valore deployato. Poi
  ripetere dopo la §2.2, dove i pesi cambiano natura.
- **Verifica** — `decision_plane.py --set mpc_W_obs_sigmoid=...` fa già esattamente questo per
  un valore alla volta; serve solo il ciclo esterno.
- **Sforzo** — **S** (lo sweep) · l'infrastruttura è pronta

---

## 6. Capitolo 3 — Formulazione del problema

### 6.1 Dichiarare l'FHOCP

- **Corso** — §3.3 eq. (3.11), con (3.11f) `h_final(z(N|t), u(N|t)) ≥ 0`; il vincolo terminale
  tipico è `f_z(z(N|t), u(N|t)) = z(N|t)`, cioè lo stato terminale è un equilibrio.
- **Stato** — l'MPC **è** la (3.11), ma manca (3.11f): c'è solo un costo terminale
  `Q_terminal = 100` moltiplicativo su Q.
- **Da aggiungere** — la scrittura formale completa del problema risolto, in notazione delle
  dispense, con la mappa esplicita fra ogni termine e la riga di codice che lo costruisce. È
  mezza pagina e va all'inizio del report: senza, nessuna delle altre sezioni ha un referente.
  Il vincolo terminale mancante è trattato nella §1.2.
- **Sforzo** — **S**

---

## 7. Piano di lavoro

### Blocco A — teoria mancante sul codice esistente *(nessuna modifica al comportamento)*

| # | Voce | §guida | Sforzo |
|---|---|---|---|
| 1 | Scrittura formale dell'FHOCP con mappa al codice | 6.1 | S |
| 2 | KKT, LICQ, moltiplicatori, SOC-C-2 | 2.1 | M |
| 3 | Classificazione del problema, non convessità dei due termini ostacolo | 5.1, 5.2 | M |
| 4 | AD vs differenze finite; Hessiana esatta vs L-BFGS; `t_proc_nlp_*` | 4.1 | S |
| 5 | Tabella sparsità NLP vs N + argomento multiple shooting | 1.5 | M |
| 6 | Sweep di W_obs: soglia di biforcazione (Fig. 4.17 sui dati veri) | 5.4 | S |
| 7 | Baseline del warm start (shiftata / riferimento / cold) | 1.7 | S |
| 8 | Errore di predizione modello vs impianto MuJoCo, dalle bag | 1.4 | S |
| 9 | Rivendicare: ZOH esatta e la sua degenerazione sul G1; MIP evitato via A\* | 3.3, 1.9 | S |

### Blocco B — riformulazioni *(cambiano il comportamento, vanno validate)*

| # | Voce | §guida | Sforzo | Dipende da |
|---|---|---|---|---|
| 10 | **Path following in θ** (elimina `v_ref`) | 1.1 | M | — |
| 11 | **Vincolo ostacolo con slack + penalità ℓ¹ esatta, ρ da μ\*** | 2.2 | M | 2 |
| 12 | **Vincolo terminale di equilibrio + fattibilità ricorsiva** | 1.2 | M | — |
| 13 | Mid-point (RK2) al posto di Euler + fit dell'ordine | 3.1 | S | — |
| 14 | Sweep dell'orizzonte N: prestazione vs tempo di calcolo | 1.3 | M | — |
| 15 | Fronte di Pareto sui dati esistenti | 1.8 | M | 10 (sinergia) |
| 16 | Omotopia su `obs_alpha` | 1.7, 2.5 | M | — |
| 17 | Move blocking / forma in incrementi | 1.6 | S/M | — |

### Blocco C — il pezzo grosso

| # | Voce | §guida | Sforzo | Dipende da |
|---|---|---|---|---|
| 18 | **SQP + Gauss-Newton vincolato scritti a mano contro IPOPT** | 2.3 | L | 2, 11 |
| 19 | Active-set contro interior-point, prima e dopo il punto 11 | 2.4 | M | 18 |
| 20 | Insieme terminale come sottolivello di Lyapunov (DARE) | 1.2 | L | 12 |

### Se il tempo è poco

I quattro interventi con il miglior rapporto contenuto/sforzo sono **13** (RK2, mezza giornata
e dà un fit d'ordine pulito), **6** (soglia di biforcazione, l'infrastruttura è già pronta),
**11** (penalità ℓ¹ esatta, è il Thm 6.3.1 dimostrato sperimentalmente) e **10** (path
following in θ, la formulazione che il corso insegna per esattamente questo problema).

Il **12** merita comunque una menzione anche senza implementarlo, perché il materiale
sperimentale esiste già ed è insolitamente concreto.

---

### Riproducibilità

Gli snippet di supporto stanno in `guides/snippets/`. `nlp_structure.py` riproduce i numeri
della §0 e accetta una lista di orizzonti per la tabella della §1.5:

```bash
python3 guides/snippets/nlp_structure.py            # N deployato
python3 guides/snippets/nlp_structure.py 10 15 25 50
```

Non richiede ROS: bastano `casadi`, `numpy`, `scipy`.

L'analisi su dati reali passa da [`viz/`](../viz/) — vedi
[`visualizzazione_ottimizzazione.md`](visualizzazione_ottimizzazione.md) per il flusso
registrazione → replay → pannelli.

---

## 8. Cosa resta fuori perimetro

| Componente | Perché è fuori |
|---|---|
| **Identificazione di τ_v, τ_w con SEM** (§3.1.2, §7.3.1) | *optimal system identification*: altra categoria di problema. Sarebbe implementabile — i bag ci sono — ma diluirebbe l'elaborato. Nota: il §0.2 mostra che i τ deployati sono degeneri, quindi un'identificazione seria cambierebbe il modello, non solo un numero |
| **Moving Horizon Estimation** (§3.2, §7.3.2) | *optimal observer*: idem. Oggi le velocità vengono da una media esponenziale sulle differenze di posa |
| **Bayesian Optimization TPE** ([`tuning/`](../tuning/), hyperopt) | ottimizzazione globale black-box: **non compare in nessun capitolo delle dispense**. Recuperabile solo come multi-obiettivo (§1.8). Dopo la §2.2 una parte del tuning diventa comunque superflua |
| **Surrogato GP, ARD Matérn-5/2** | machine learning |
| **PointCloud-GNNencoder (DGCNN)** | deep learning; unico aggancio §7.2.5 (§1.10), non vale svilupparlo |
| [`robot_nav/`](../src/robot_nav/) — grafo topologico, Dijkstra | ricerca su grafo discreta |
| [`persistent_map.py`](../src/a_star_mpc_planner/a_star_mpc_planner/persistent_map.py) | mappatura |
| A\* + griglia gaussiana | ricerca combinatoria — ma **una riga è dovuta**: è ciò che evita il MIP (§1.9) e genera z̄(θ) per la §1.1 |
| `champ*`, `go2_*`, `g1_sim`, `unitree_api/go`, `d1_sim`, `sensor_models`, `sim_worlds`, `robot_safety`, `robot_sim` | ingegneria robotica e infrastruttura: mezza pagina di contesto, zero contenuto teorico |
| Euristiche in `mpc_node.py`: escape BFS, limiti adattivi, staleness LiDAR, low-pass del setpoint, predizione ostacoli dinamici | patch di deployment — **da citare onestamente**: sono i sintomi dei buchi delle §1.2 e §2.2. Presentarle come *"cosa abbiamo fatto in assenza di ingredienti terminali e di vincoli soft"* è più forte che ometterle, e ora c'è la misura (§1.2) |

---

## 9. Indice inverso: dispense → voce di questa guida

| Sezione delle dispense | Voce |
|---|---|
| §2.1.3 discretizzazione, eq. (2.9)–(2.10) | **3.1**, 3.2, 3.3 |
| §3.3 FHOCP eq. (3.11) | **6.1**, 1.2 |
| §3.3 FHOCP parametrizzato eq. (3.13) | 8 (BO) |
| §4.1 esistenza, Thm 4.1.1 | 5.1 |
| §4.2 classificazione LP/QP/convesso/MIP | 5.1, **1.9** |
| §4.2.5 non liscio | 5.1 |
| §4.3 convessità, Thm 4.3.1–4.3.3 | **5.2** |
| §4.4.1 NOC/SOC | 2.1 |
| §4.4.2 line search, Armijo, Thm 4.4.5 | 5.3, 2.3 |
| §4.4.3 tassi di convergenza, condizionamento | 5.3, 2.3 |
| §4.4.4 Newton, Gauss-Newton, BFGS | 5.3, 4.1, **2.3** |
| §4.4.5 regolarità di x\*(ϑ), Thm 4.4.6, Fig. 4.17 | **5.4** |
| §5.1 ricorsione delle sensitività eq. (5.3) | 4.2 |
| §5.2 differenze finite | **4.1** |
| §5.3 AD, CasADi | **4.1** |
| §6.1.1 cono tangente, LICQ | **2.1** |
| §6.1.2 KKT eq. (6.8), Farkas | **2.1** |
| §6.1.3 cono critico, SOC-C-2 | **2.1** |
| §6.2.1 QP con uguaglianze, sistema KKT | 2.4 |
| §6.2.2 active-set vs interior-point, barriera log | **2.4**, 2.5 |
| §6.3.1 SQP Hessiana esatta | **2.3** |
| §6.3.2 Gauss-Newton vincolato, BFGS + Powell | **2.3** |
| §6.3.3 merit ℓ¹, Thm 6.3.1 penalità esatta | **2.2**, 2.3 |
| §6.3.4 Algoritmo 6.3.1 | **2.3** |
| §7.1.1 omotopia e warm starting eq. (7.2) | **1.7** |
| §7.2.2 single vs multiple shooting | **1.5** |
| §7.2.3 parametrizzazione, move blocking | **1.6** |
| §7.2.4 path following eq. (7.5) | **1.1** ⭐ |
| §7.2.5 MPC pratico, fattibilità ricorsiva | **1.2** ⭐, 1.3, 1.4, 1.10 |
| §7.4 multi-obiettivo, Pareto, Utopico | **1.8** |
| §3.1.2, §7.3.1 SEM | fuori perimetro (§8) |
| §3.2, §7.3.2 MHE | fuori perimetro (§8) |

---

## 10. Stato di avanzamento

Sezione di riferimento per il gruppo: **cosa è già implementato e misurato**, con la lettura
dei risultati, e **cosa resta da fare**. Aggiornata al 21 agosto 2026.

Le tre voci fatte sono state scelte perché la prima è verificabile in mezza giornata, la
seconda sblocca la terza, e la terza è il risultato più caratterizzante per l'esame.

### 10.1 Fatto — riepilogo

| # | Voce | §guida | Dove sta il codice | Come si rilancia |
|---|---|---|---|---|
| 13 | Integratore RK2 (punto medio) | 3.1 | `MPCConfig.integrator`, param ROS `mpc_integrator` | `python3 tests/test_integrators.py` |
| 2 | KKT, LICQ, complementarità, SOC-C-2 | 2.1 | `viz/kkt_analysis.py` | `python3 viz/kkt_analysis.py --bag viz/bags/industrial_plant_fix` |
| 11 | Penalità esatta ℓ¹ con slack | 2.2 | `MPCConfig.obstacle_mode`, `viz/exact_penalty.py` | `python3 viz/exact_penalty.py --scenario narrow_gap --d-safe 1.1` |
| — | Struttura e sparsità dell'NLP | 0, 1.5 | `guides/snippets/nlp_structure.py` | `python3 guides/snippets/nlp_structure.py 10 15 25 50` |
| — | Pannelli di visualizzazione + replay da bag | — | `viz/` | vedi [`visualizzazione_ottimizzazione.md`](visualizzazione_ottimizzazione.md) |

**Il comportamento deployato non è cambiato.** `obstacle_mode` resta `penalty` e il solve su un
ciclo reale dà `J*` **bit-identico** al valore pre-modifiche (8177.231314839336), con gli stessi
156 vincoli e nessuna variabile di slack allocata. L'unica modifica al comportamento è
`mpc_integrator: 'midpoint'` nel profilo G1, ed è reversibile cambiando una riga di YAML.

---

### 10.2 Integratore: ordine verificato, ma il guadagno in anello chiuso è quasi nullo

`tests/test_integrators.py` confronta i due schemi con la soluzione esatta in forma chiusa
(per ω ≠ 0 lo spostamento è un arco di circonferenza) e ne stima l'ordine con un fit log-log.

**Ordine 1.00 per Euler e 2.00 per il punto medio**, identico su tre regimi. Al `dt = 0.2`
deployato, su 3 s di predizione: Euler sbaglia **17.4 mm**, il punto medio **0.087 mm**. Un
fattore 200, al prezzo di una singola addizione.

Il test verifica anche che la dinamica costruita **dentro l'NLP** coincida con lo schema di
riferimento (scarto 0.000e+00): non basta che torni la teoria, deve integrare così `mpc_tracker`.

**Il risultato interessante è però negativo.** In anello chiuso, su quattro scenari, il costo
mediano migliora dello 0.0–1.1 % e distanza finale e clearance restano identiche. La ragione è
strutturale e va detta nel report: a orizzonte recedente si applica solo il **primo** ingresso e
A\* ripianifica, quindi l'errore di predizione accumulato sui 15 passi non si trasferisce alla
traiettoria percorsa. *(Il confronto è pulito: l'impianto simulato usa un aggiornamento di Euler
fisso indipendente da `cfg.integrator`; cambia solo il modello di predizione.)*

Il valore del punto medio sta quindi nella **fedeltà della predizione**, non nella traiettoria.
Diventa rilevante nella §1.3, se si allunga l'orizzonte, e nella §1.4, dove serve poter
attribuire l'errore di predizione al disallineamento di modello e non all'integratore.

---

### 10.3 KKT: il problema è molto più *vincolato* di quanto sembri

`viz/kkt_analysis.py` verifica su cicli reali estratti dalle bag ciò che le dispense enunciano
in astratto. Su nove cicli di `industrial_plant_fix`:

- **LICQ vale in ogni ciclo** (rango del Jacobiano attivo = numero di vincoli attivi) ⇒ i
  moltiplicatori KKT esistono e sono **unici**;
- **complementarità stretta in ogni ciclo** ⇒ il cono critico coincide con il nucleo del
  Jacobiano attivo, e SOC-C-2 si verifica **esattamente** anziché per difetto;
- **SOC-C-2 soddisfatta ovunque** (λ_min > 0 sull'Hessiana proiettata) ⇒ certificato di minimo
  locale stretto, che è il massimo ottenibile in un problema non convesso;
- stazionarietà: `‖∇_x L(x*, λ*)‖_∞ = 6.1e-11` contro `‖∇_x f(x*)‖_∞ = 1.6e+04`.

**Il risultato da raccontare** è la dimensione del cono critico, che **collassa da 45 a 1**
lungo la missione. A fine corsa ci sono 141 variabili e 140 vincoli attivi: resta un solo grado
di libertà. L'MPC passa da *guidato dal costo* a **guidato dai vincoli**, e nella fase finale
non sceglie più la traiettoria, la subisce.

La ripartizione dei vincoli attivi lo spiega: `|vy| ≤ vy_max` e `|ω| ≤ ω_max` sono saturi
**15 volte su 15** nei cicli finali, `vx ≥ 0` 14 su 15. È la conferma quantitativa che
`vy_max = 0.02` m/s non *limita* un grado di libertà: lo **rimuove**.

> **Nota per chi riusa il codice.** Due trappole di CasADi Opti, entrambe incontrate qui:
> 1. Opti canonizza ogni vincolo come `lbg ≤ g(x) ≤ ubg` e **assorbe nei limiti** il membro
>    destro quando è un parametro: `X[:,0] == p_x0` diventa `g = X[:,0]`, `lbg = ubg = p_x0`.
>    Valutare `|g|` su quelle righe restituisce **lo stato**, non il residuo. Il residuo è
>    sempre `g − lbg`.
> 2. Due righe distinte possono condividere la stessa espressione con limiti diversi
>    (`U[0,k] ≥ 0` e `U[0,k] ≤ vx_max` sono **entrambe** la riga `U[0,k]`). L'attivazione va
>    quindi decisa sulla **distanza dal limite**, non su `|g| ≈ 0`, altrimenti si contano
>    attivi tutti e due i lati dello stesso box.

---

### 10.4 Penalità ℓ¹: Thm 6.3.1 verificato, e due difetti di formulazione trovati per strada

`MPCTracker` accetta ora `obstacle_mode` ∈ {`penalty`, `l1`, `l2`}: nei due modi nuovi
l'ostacolo diventa un **vincolo vero** rilassato con slack, e lo slack è penalizzato in norma 1
oppure al quadrato.

Scenario `narrow_gap` con `d_safe = 1.1` m — scelto perché il vincolo **morda**: la clearance
naturale è 0.76 m. Moltiplicatore letto dal problema: **max|μ\*| = 8.905e+03**.

| ρ | max s\* (ℓ¹) | max s\* (ℓ²) |
|---|---|---|
| 1e+03 | 1.457e-01 | 2.606e-01 |
| **1e+04** | **0** | 1.297e-01 |
| 1e+05 | **0** | 3.975e-02 |
| 1e+06 | **0** | 4.375e-03 |
| 1e+08 | **0** | 4.424e-05 |

- **ℓ¹**: lo slack diventa **esattamente nullo** da ρ = 1e4 in su, e la soglia teorica
  max|μ\*| = 8.9e3 cade **esattamente** fra 1e3 (dove s\* = 0.146) e 1e4.
- **ℓ²**: il residuo decresce con pendenza log-log **−1.00 esatta** sulla coda e **non si
  annulla mai** — 4.4e-05 ancora a ρ = 1e8.

È il Thm 6.3.1 dimostrato sperimentalmente, in una tabella sola. La conseguenza pratica è che
**ρ smette di essere un iperparametro**: si legge dai moltiplicatori del problema.

**Due difetti di formulazione emersi implementandolo**, entrambi con contenuto teorico:

1. **Il vincolo non va imposto a k = 0.** Lì lo stato è fissato da `X[:,0] == x0`, quindi il
   vincolo non dipende da alcuna variabile decisionale: imporlo rende l'NLP inammissibile ogni
   volta che il robot si trova **già** entro `d_safe` da un ostacolo, cioè proprio quando
   servirebbe. Misurato sulla bag: con `d_safe = 0.9` m lo slack saturava a 0.1999, che è
   esattamente la violazione a k = 0 (clearance corrente 0.7017 m). Il vincolo parte ora da
   k = 1 e lo slack ha N colonne, non N+1.
2. **L'inammissibilità residua è colpa di U_Σ, non dei dati.** Anche partendo da k = 1, sul
   ciclo reale `d_safe = 0.75` m resta inammissibile: con `vx ≥ 0` e `vy_max = 0.02` m/s il
   robot in un passo può solo avanzare di 0.06 m lungo la propria direzione, e non può
   arretrare lateralmente dal muro di punti LiDAR. È l'interazione fra l'asimmetria
   dell'insieme ammissibile degli ingressi (§5.1) e un vincolo di stato, e collega questa
   scheda alla §1.2 (fattibilità ricorsiva).

**Costo da mettere in conto se si vuole deployare questo modo**: le disuguaglianze passano da
60 a oltre 300 e le iterazioni da ~20 a 60–240. Il cap deployato `mpc_max_iter = 40` non basta.
Questo **ribalta l'argomento della §2.4** (active-set contro interior-point) esattamente come
la scheda prevedeva, e va rimisurato prima e dopo.

---

### 10.5 Da fare, in ordine di priorità

Il criterio: prima ciò che ha l'infrastruttura già pronta, poi ciò che cambia la formulazione,
infine il pezzo grosso.

**Subito, infrastruttura già pronta**

| # | Voce | §guida | Perché ora |
|---|---|---|---|
| 6 | Sweep di W_obs: soglia di biforcazione | 5.4 | `decision_plane.py --set` fa già il singolo punto; manca solo il ciclo esterno. Dà la Fig. 4.17 sui dati propri |
| 4 | AD vs differenze finite, `t_proc_nlp_*` | 4.1 | CasADi restituisce già le statistiche in `sol.stats()`; oggi ne leggiamo solo `iter_count` |
| 8 | Errore di predizione modello vs impianto MuJoCo | 1.4 | `/mpc/predicted_path` e `/robot_pose` sono già nelle bag: è sola analisi, nessun nuovo esperimento |

**Poi, riformulazioni da validare**

| # | Voce | §guida | Nota |
|---|---|---|---|
| 10 | Path following in θ (elimina `v_ref`) | 1.1 | ⭐ la formulazione che il corso insegna per esattamente questo problema |
| 12 | Vincolo terminale di equilibrio | 1.2 | ⭐ il materiale sperimentale esiste già (il latch, §1.2) |
| 14 | Sweep dell'orizzonte N | 1.3 | l'orizzonte copre 0.60 m: va giustificato o cambiato |
| 15 | Fronte di Pareto | 1.8 | sinergia con il 10, che introduce già tre obiettivi pesati |

**Infine**

| # | Voce | §guida | Nota |
|---|---|---|---|
| 18 | SQP + Gauss-Newton scritti a mano contro IPOPT | 2.3 | il più corposo e il più caratterizzante; ora ha i μ\* di riferimento dal punto 2 |
| 19 | Active-set contro interior-point | 2.4 | da rifare **prima e dopo** il punto 11, vedi §10.4 |

**Non ancora toccato e da decidere**: la scrittura formale dell'FHOCP (§6.1, punto 1) — è mezza
pagina ma va all'inizio del report, perché senza di essa nessuna delle altre sezioni ha un
referente esplicito.

---

### 10.6 Come verificare che tutto giri ancora

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

python3 tests/test_integrators.py        # ordine 1.00 / 2.00, scarto NLP 0.000e+00
python3 viz/test_fidelity.py             # costo replicato == opti.f, errore 0.000e+00
python3 guides/snippets/nlp_structure.py # struttura dell'NLP deployato
python3 viz/kkt_analysis.py   --bag viz/bags/industrial_plant_fix
python3 viz/exact_penalty.py  --scenario narrow_gap --d-safe 1.1
python3 viz/cost_field.py     --bag viz/bags/industrial_plant_fix
python3 viz/decision_plane.py --bag viz/bags/industrial_plant_fix
```

Nessuno di questi richiede il robot o il simulatore: girano tutti su bag registrate o su
scenari sintetici. Per registrare una nuova missione, vedi
[`visualizzazione_ottimizzazione.md`](visualizzazione_ottimizzazione.md).
