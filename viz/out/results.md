# Risultati — generati automaticamente

> Non modificare a mano: rigenerare con `python3 viz/make_results.py`.

- data: 2026-08-22T13:21:40+00:00
- commit: `5d8c55dc2d` sul branch `G1_optimal_trajectory`  **(albero di lavoro sporco: numeri non riproducibili da questo commit)**
- profilo: `src/a_star_mpc_planner/config/planner_params_g1.yaml`
- CasADi 3.7.2, numpy 1.26.4, Python 3.10.12

Parametri: N=15, dt=0.2, v_ref=0.2, vx_max=0.3, W_obs=120.0, integrator=midpoint, path_mode=time, terminal=none


## Classe 1 — proprietà della formulazione

*Indipendenti dalla run: si calcolano una volta sola.*


### Ordine di troncamento (§2.1.3)

| regime | ordine Euler | ordine punto medio |
|---|---|---|
| nominale (vx=0.2, w=0.3) | 1.00 | 2.00 |
| con deriva laterale | 1.00 | 2.00 |
| rotazione rapida (w=1.0) | 1.00 | 2.00 |

Al dt deployato (0.2) su 3 s: Euler 1.740e-02 m, punto medio 8.700e-05 m — guadagno 200×.


### Derivate: AD contro differenze finite (§5.2–5.3)

| metodo | valutazioni di f | accuratezza |
|---|---|---|
| differenze in avanti | 142 | 5.5e-08 |
| differenze centrate | 282 | 1.3e-10 |
| **AD inverso** | **1.1** (intervallo 0.8–1.8) | precisione macchina |

*Il costo dell'AD è un micro-benchmark su tempi di ~100 μs: si riporta la mediana di più misure con il suo intervallo, perché una singola coppia oscilla sensibilmente. Quello che conta, ed è stabile, è che stia fra 1 e 3 come prevede il §5.3 — non la sua seconda cifra.*

> **Misura non attendibile**: qualche ripetizione ha dato un rapporto < 1, cioè un gradiente più veloce della funzione. Rieseguire a macchina scarica prima di usare questo numero.

Passi ottimi misurati: avanti 1.49e-08 (teorico √eps = 1.49e-08), centrate 6.06e-06 (teorico eps^(1/3) = 6.06e-06).
Le differenze centrate userebbero il 40% del budget di ciclo (125 ms).


### Hessiana esatta contro L-BFGS (§4.4.4)

| Hessiana | iterazioni | J* |
|---|---|---|
| exact | 20 | 9244.061 |
| limited-memory | 36 | 9244.061 |

### Penalità esatta ℓ¹ (Thm 6.3.1)

d_safe = 1.1, max|μ\*| = 8.905e+03

| ρ | slack ℓ¹ | slack ℓ² |
|---|---|---|
| 1e+03 | 1.457e-01 | 2.606e-01 |
| 1e+04 | 0 | 1.297e-01 |
| 1e+06 | 0 | 4.375e-03 |

ℓ¹ nullo da ρ = 1e+04; pendenza ℓ² sulla coda = -0.61 (attesa −1).


### Struttura dell'NLP

| N | variabili | vincoli | densità jac | densità hess |
|---|---|---|---|---|
| 10 | 96 | 106 | 2.52% | 6.51% |
| 15 | 141 | 156 | 1.73% | 4.45% |
| 25 | 231 | 256 | 1.07% | 2.73% |


## Classe 2 — proprietà dell'istanza

*Variano ciclo per ciclo: il dato è il profilo, non un numero singolo.*


### KKT lungo la missione (§6.1)

LICQ sempre verificata: **True** · complementarità stretta sempre: **True** · SOC-C-2 sempre soddisfatta: **True**

Dimensione del cono critico fra **1** e **45** a seconda del punto di lavoro (vale l'identità `dim(cono) = n_var − vincoli attivi`: è il complemento della saturazione, non una tendenza temporale).

| ciclo | t [s] | attivi | rango | LICQ | cono | λ_min proiettato |
|---|---|---|---|---|---|---|
| 0 | 0 | 96 | 96 | sì | 45 | +8.78e-01 |
| 193 | 52 | 140 | 140 | sì | 1 | +7.18e+02 |
| 387 | 110 | 120 | 120 | sì | 21 | +9.73e-01 |
| 580 | 203 | 137 | 137 | sì | 4 | +2.34e+00 |
| 774 | 299 | 133 | 133 | sì | 8 | +1.32e+00 |

### Biforcazione (§4.4.5, Thm 4.4.6)

Soglia fra W_obs = 200 e 300; il deployato è 120 (sotto soglia).
Sul ciclo reale 718: biforca mai = **False**.



## Classe 3 — prestazione in anello chiuso

*Dipendono da run e mondo: qui servono più missioni.*


### Errore di predizione (§7.2.5) — bag `industrial_plant_fix`, 775 cicli

Offset a k=0: 0.0244 m (allineamento temporale, non modello).
**Divergenza a fine orizzonte: 0.124 m**, cioè 7× l'errore di Euler e 1428× quello del punto medio.


### Path following in θ (§7.2.4)

| grandezza | riferimento a tempo | ascissa θ |
|---|---|---|
| vx media [m/s] | 0.2130 | 0.3000 |
| spostamento [m] | 0.6358 | 0.8996 |
| iterazioni | 11.2 | 21.0 |

**+41% di avanzamento**; v_ref lasciava inutilizzato il 33% della velocità.


### Vincolo terminale (§7.2.5)

Slack massimo 0.000e+00 — sempre ammissibile: **True**. Costo del vincolo da +0.2% a +85.0%.

