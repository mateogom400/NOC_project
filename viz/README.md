# viz — visualizzazione del problema di ottimizzazione

Due strumenti offline, documentati in
[`guides/visualizzazione_ottimizzazione.md`](../guides/visualizzazione_ottimizzazione.md).

```bash
python3 viz/test_fidelity.py        # il costo disegnato == il costo ottimizzato
python3 viz/cost_field.py           # pannello 1: paesaggio di navigazione c(x,y)
python3 viz/decision_plane.py       # pannello 2: spazio delle decisioni + iterati IPOPT
```

Non richiedono ROS: bastano `casadi`, `numpy`, `scipy`, `matplotlib`, `pyyaml`.
Le figure finiscono in `viz/out/`.
