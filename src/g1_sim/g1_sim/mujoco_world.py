"""
mujoco_world — build the MuJoCo model for navigation simulation.

Loads the G1 MJCF (which already has a free `floating_base_joint` and resolves
its own meshes) and augments its spec with:
  - the industrial warehouse geometry, replicated from sim/worlds/industrial.sdf,
    placed in geom group 3 so the simulated LiDAR can ray-cast against ONLY the
    environment (the robot's own body is excluded → no self-mapping);
  - a floor plane + lights (group 0, not seen by the LiDAR);
  - a `mid360` site on torso_link matching the URDF LiDAR mount, used as the
    ray origin (so the published cloud is consistent with the mid360_link TF).

Everything stays in a single MjSpec, so mesh paths from the G1 file remain valid.

SDF→MuJoCo conversions:
  - box <size> is a FULL extent in SDF but a HALF extent in MuJoCo.
  - cylinder: SDF <length> is full; MuJoCo size is [radius, half_length].
"""

import math
import numpy as np
import mujoco

# Geom group cast by the LiDAR (environment only; robot/floor excluded).
LIDAR_GROUP = 3

# mid360 mount on torso_link (from 29dof.urdf: pos + 0.04 rad pitch)
MID360_POS = (0.0002835, 3e-05, 0.40618)
MID360_PITCH = 0.04014257279586953


def _yaw_quat(yaw):
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def _box(x, y, z, sx, sy, sz, rgba, yaw=0.0, group=None):
    """SDF box (full sizes) → MuJoCo box dict (half sizes).

    `group` a None significa LIDAR_GROUP, cioe' un ostacolo vero. Passando 0 si
    ottiene una geometria di SOLA DECORAZIONE: visibile nel viewer ma esclusa
    dal ray-cast, quindi invisibile al pianificatore. Serve per i marcatori di
    goal, che altrimenti sarebbero ostacoli piazzati esattamente dove il robot
    deve arrivare.
    """
    return dict(shape="box", pos=[x, y, z], size=[sx / 2, sy / 2, sz / 2],
                rgba=rgba, yaw=yaw, group=group)


def _marker(x, y, rgba):
    """Disco piatto a terra che segna un punto notevole. Decorazione pura:
    group 0, e comunque sotto z_min del filtro (0.15 m)."""
    return dict(shape="cyl", pos=[x, y, 0.01], size=[0.35, 0.01],
                rgba=rgba, yaw=0.0, group=0)


def _cyl(x, y, z, radius, length, rgba, group=None):
    return dict(shape="cyl", pos=[x, y, z], size=[radius, length / 2], rgba=rgba,
                yaw=0.0, group=group)


# Material colours (approx. of industrial.sdf)
_WALL = [0.8, 0.8, 0.8, 1]
_COL = [0.4, 0.4, 0.5, 1]
_RACK = [0.6, 0.4, 0.2, 1]
_PALLET = [0.7, 0.55, 0.2, 1]
_BOXC = [0.3, 0.5, 0.7, 1]
_GREEN = [0.35, 0.6, 0.4, 1]
_CONV = [0.30, 0.30, 0.34, 1]
_ARM = [0.9, 0.45, 0.1, 1]
_DARK = [0.2, 0.2, 0.2, 1]
_SHELF = [0.55, 0.4, 0.25, 1]
_FORK = [0.85, 0.7, 0.1, 1]


def _seg(x1, y1, x2, y2, height=2.5, thick=0.25, rgba=None):
    """Muro fra due punti del piano: e' il modo naturale di disegnare
    geometrie non convesse (una U e' tre segmenti, un corridoio quattro).

    Restituisce un box centrato a meta' segmento, lungo quanto il segmento e
    ruotato per allinearvisi. Altezza e quota sono scelte perche' il muro cada
    dentro la fascia che il filtro LiDAR tiene (z_min 0.15, z_max 1.60 nel
    frame odom, vedi config/lidar_filter_g1.yaml): un ostacolo tutto sopra o
    tutto sotto quella fascia verrebbe scartato e il pianificatore non lo
    vedrebbe mai.
    """
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    return dict(shape="box",
                pos=[(x1 + x2) / 2.0, (y1 + y2) / 2.0, height / 2.0],
                size=[length / 2.0, thick / 2.0, height / 2.0],
                rgba=rgba or _WALL, yaw=math.atan2(dy, dx))


def _arena(hx, hy, height=3.0, thick=0.2):
    """Quattro muri perimetrali: il robot non puo' uscire dal mondo e la
    scappatoia attorno a un ostacolo resta una scelta, non una fuga."""
    return [_seg(-hx, hy, hx, hy, height, thick),
            _seg(-hx, -hy, hx, -hy, height, thick),
            _seg(-hx, -hy, -hx, hy, height, thick),
            _seg(hx, -hy, hx, hy, height, thick)]


def warehouse_geoms():
    """Return the list of obstacle geoms (replica of industrial.sdf)."""
    g = []
    # Perimeter walls
    g += [_box(0, 10, 1.5, 30, 0.2, 3, _WALL), _box(0, -10, 1.5, 30, 0.2, 3, _WALL),
          _box(15, 0, 1.5, 0.2, 20, 3, _WALL), _box(-15, 0, 1.5, 0.2, 20, 3, _WALL)]
    # Columns
    for cx in (-10, 0, 10):
        for cy in (6, -6):
            g.append(_cyl(cx, cy, 1.5, 0.15, 3.0, _COL))
    # Racks
    for (rx, ry) in [(-12, 7.5), (4, 7.5), (-12, -7.5), (4, -7.5)]:
        g.append(_box(rx, ry, 1.25, 6, 0.6, 2.5, _RACK))
    # Pallets + box
    g += [_box(6, 2, 0.075, 1.2, 0.8, 0.15, _PALLET),
          _box(6, 2, 0.475, 0.6, 0.4, 0.5, _BOXC, yaw=0.3),
          _box(7, -3, 0.075, 1.2, 0.8, 0.15, _PALLET)]
    # Conveyors
    g += [_box(-1, 4.3, 0.35, 8, 0.7, 0.7, _CONV), _box(1, -4.3, 0.35, 8, 0.7, 0.7, _CONV)]
    # Robotic-arm workcells (base cyl + column cyl + arm box)
    for (ax, ay) in [(-1, 3.3), (1, -3.3)]:
        g += [_cyl(ax, ay, 0.25, 0.30, 0.5, _DARK),
              _cyl(ax, ay, 1.05, 0.12, 1.1, _ARM),
              _box(ax, ay, 1.5, 0.9, 0.18, 0.18, _ARM)]
    # East shelves
    g += [_box(13, 4, 1.3, 0.6, 5, 2.6, _SHELF), _box(13, -4, 1.3, 0.6, 5, 2.6, _SHELF)]
    # Pallet staging
    g += [_box(-6, -4, 0.075, 1.2, 0.8, 0.15, _PALLET),
          _box(-6, -4, 0.55, 0.9, 0.7, 0.8, _BOXC),
          _box(-7.4, -4, 0.075, 1.2, 0.8, 0.15, _PALLET),
          _box(-7.4, -4, 0.40, 0.8, 0.6, 0.5, _GREEN)]
    # Crates
    g += [_box(8, 5.5, 0.3, 0.6, 0.6, 0.6, _BOXC, yaw=0.4),
          _box(8.7, 6, 0.45, 0.5, 0.5, 0.9, _GREEN),
          _box(-3, -6.8, 0.3, 0.6, 0.6, 0.6, _BOXC, yaw=0.8)]
    # Forklift (body + cabin + mast), approx at (10, 3.5) yaw 1.2
    g += [_box(10, 3.5, 0.35, 1.1, 0.7, 0.7, _FORK, yaw=1.2),
          _box(10, 3.5, 1.05, 0.6, 0.6, 0.7, _FORK, yaw=1.2),
          _box(10, 3.5, 1.0, 0.1, 0.6, 2.0, _DARK, yaw=1.2)]
    return g


# ---------------------------------------------------------------------------
# Mondi con ostacoli NON CONVESSI
# ---------------------------------------------------------------------------
# Il magazzino industriale e' fatto di ostacoli convessi e sparsi: A* ne esce
# sempre con una deviazione locale, e il pianificatore non viene mai messo
# davanti a un minimo locale vero. Questi tre mondi servono a quello.
#
# La misura che li rende non banali e' la FINESTRA DI A*: grid_half_width = 6.0
# significa 12x12 m centrati sul robot, e il LiDAR arriva a 8 m. Un ostacolo
# concavo piu' PICCOLO della finestra non e' una trappola — A* ne vede subito
# il contorno completo e lo aggira. Diventa una trappola solo quando la via
# d'uscita cade FUORI dalla finestra, cioe' quando il pianificatore deve
# decidere sapendo di non vedere abbastanza. Le quote qui sotto sono scelte
# per stare da quel lato.
#
# Tutti i muri sono alti 2.5 m: la fascia utile del filtro e' z in [0.15, 1.60]
# in frame odom, quindi la geometria e' vista per intero a ogni quota di
# scansione, senza dipendere dal beccheggio del busto.


def long_wall_geoms():
    """Muro lungo, con il goal esattamente dietro.

    Il muro va da y=-8 (attaccato al perimetro sud) a y=+3.5, quindi e' lungo
    11.5 m e l'unico passaggio e' il varco di 4.5 m fra la sua estremita' nord
    e il muro perimetrale. Dallo spawn quel varco e' a 8.5 m di distanza in y:
    FUORI dalla finestra 12x12 m di A* finche' il robot non risale. E' la
    versione minima del problema "il percorso ottimo comincia allontanandosi
    dal goal".
    """
    g = _arena(12.0, 8.0)
    g += [_seg(0.0, -8.0, 0.0, 3.5, 2.5, 0.30)]
    # Riferimenti visivi ai due lati, per capire a colpo d'occhio nel viewer
    # da che parte del muro si trova il robot.
    g += [_marker(-6.0, -4.0, [0.2, 0.4, 0.9, 1]),   # spawn
          _marker(6.0, -4.0, [0.2, 0.8, 0.3, 1])]    # goal
    return g


def horseshoe_geoms():
    """Trappola a U (ferro di cavallo) aperta verso il robot, goal oltre il fondo.

    Il fondo e' a x=+3 e i bracci arrivano a x=-2, quindi la U e' profonda 5 m e
    larga 7 m: il robot che punta il goal in linea retta ci entra dentro. E' il
    minimo locale classico dei campi di potenziale, e qui e' un test della
    memoria persistente degli ostacoli di a_star_node (`_persistent_map`), che
    esiste proprio per non dimenticare i bracci una volta usciti dal campo
    visivo — senza, il robot rientra nella U all'infinito.

    I bracci sono lunghi 5 m, meno degli 8 m di portata del LiDAR: la U e' vista
    per intero da dentro. La trappola non e' percettiva, e' geometrica.
    """
    g = _arena(12.0, 8.0)
    g += [_seg(3.0, -3.5, 3.0, 3.5, 2.5, 0.30),      # fondo
          _seg(-2.0, 3.5, 3.0, 3.5, 2.5, 0.30),      # braccio nord
          _seg(-2.0, -3.5, 3.0, -3.5, 2.5, 0.30)]    # braccio sud
    g += [_marker(-7.0, 0.0, [0.2, 0.4, 0.9, 1]),
          _marker(7.0, 0.0, [0.2, 0.8, 0.3, 1])]
    return g


def dead_end_geoms():
    """Corridoio stretto e lungo, CHIUSO in fondo, con il goal appena oltre.

    Il corridoio e' largo 2.0 m e lungo 8 m, imboccatura a x=-2 e fondo chiuso a
    x=+6. Il goal sta a (9, 0), cioe' subito dietro il fondo: la direzione del
    corridoio punta al goal, ed e' questo che lo rende una trappola convincente
    invece che un ostacolo qualunque.

    Il fondo chiuso e' a 8 m dall'imboccatura, cioe' AL LIMITE della portata del
    LiDAR (max_lidar_range 8.0) e appena dentro la finestra di A*: entrando, il
    robot non ha ancora la prova che sia chiuso. Deve percorrerlo, scoprirlo e
    tornare indietro — che e' il caso in cui la retromarcia appena riabilitata
    (mpc_vx_min = -0.15) serve davvero.

    Larghezza 2.0 m scelta di proposito: con grid_std 0.31 e obstacle_threshold
    0.10 il raggio di blocco implicato e' 0.397 m, quindi restano 1.2 m di
    canale libero. Il corridoio e' percorribile, e il test riguarda il vicolo
    cieco, non la strettoia.
    """
    g = _arena(12.0, 8.0)
    g += [_seg(-2.0, 1.0, 6.0, 1.0, 2.5, 0.30),      # parete nord
          _seg(-2.0, -1.0, 6.0, -1.0, 2.5, 0.30),    # parete sud
          _seg(6.0, -1.0, 6.0, 1.0, 2.5, 0.30)]      # FONDO CHIUSO
    # Il giro largo e' libero: la soluzione esiste, e passa a nord o a sud.
    g += [_marker(-6.0, 0.0, [0.2, 0.4, 0.9, 1]),
          _marker(9.0, 0.0, [0.2, 0.8, 0.3, 1])]
    return g


# name -> geometria, posa di spawn suggerita, goal suggerito
# spawn e' (x, y, yaw); goal e' puramente informativo (stampato all'avvio), il
# goal vero lo si assegna da RViz con 2D Goal Pose.
WORLDS = {
    "industrial": dict(geoms=warehouse_geoms, spawn=(-12.0, 0.0, 0.0),
                       goal=(10.0, 0.0),
                       desc="magazzino industriale (ostacoli convessi, sparsi)"),
    "long_wall":  dict(geoms=long_wall_geoms, spawn=(-6.0, -4.0, 0.0),
                       goal=(6.0, -4.0),
                       desc="muro lungo 11.5 m, goal dietro, varco solo a nord"),
    "horseshoe":  dict(geoms=horseshoe_geoms, spawn=(-7.0, 0.0, 0.0),
                       goal=(7.0, 0.0),
                       desc="trappola a U aperta verso il robot, goal oltre il fondo"),
    "dead_end":   dict(geoms=dead_end_geoms, spawn=(-6.0, 0.0, 0.0),
                       goal=(9.0, 0.0),
                       desc="corridoio 2.0x8 m chiuso in fondo, goal appena oltre"),
}


def world_names():
    return sorted(WORLDS)


def world_info(name):
    if name not in WORLDS:
        raise ValueError(
            f"mondo sconosciuto: {name!r}. Disponibili: {', '.join(world_names())}")
    return WORLDS[name]


def _add_person(wb, idx, color):
    """Add one ~1.7 m humanoid silhouette as a MOCAP body (legs+torso+head).

    A mocap body is moved every step by writing data.mocap_pos/mocap_quat (no
    joint, no dynamics) — the kinematic-teleport equivalent of the Gazebo
    set_pose people. Its geoms live in LIDAR_GROUP so the simulated Mid-360
    ray-casts against them (the MPC + tracker then see a moving obstacle), and
    are visual-only (contype=conaffinity=0), matching the warehouse geoms.
    Returns the body name (mocap id is resolved after compile)."""
    name = f"person_{idx}"
    body = wb.add_body(name=name, mocap=True, pos=[0.0, 0.0, -5.0])
    parts = [
        # (pos_z, type, size)  — sizes are MuJoCo half-extents
        (0.45, mujoco.mjtGeom.mjGEOM_BOX, [0.15, 0.15, 0.45]),       # legs
        (1.15, mujoco.mjtGeom.mjGEOM_BOX, [0.225, 0.14, 0.325]),     # torso
        (1.62, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.13, 0.13, 0.0]),   # head
    ]
    for pz, gtype, size in parts:
        gg = body.add_geom(type=gtype, size=size, pos=[0.0, 0.0, pz], rgba=color)
        gg.group = LIDAR_GROUP
        gg.contype = 0
        gg.conaffinity = 0
    return name


def build_model(g1_xml_path, n_people=0, people_colors=None, world="industrial"):
    """Build the combined MuJoCo model (G1 + world). Returns (model, info).

    `world` sceglie la geometria fra quelle di WORLDS (industrial, long_wall,
    horseshoe, dead_end). info["world"] e info["world_spawn"] riportano la
    scelta al chiamante, cosi' mujoco_sim puo' posizionare il robot dove quel
    mondo ha senso senza che l'utente debba ricordarsi le coordinate.

    If n_people > 0, that many mocap "person" bodies are added (parked below the
    floor at z=-5 until the sim places them); mujoco_sim teleports them along
    line/circle patterns. people_colors is an optional list of [r,g,b] used
    cyclically for the silhouettes."""
    spec = mujoco.MjSpec.from_file(g1_xml_path)
    wb = spec.worldbody

    # The G1 MJCF already provides an (infinite) `floor` plane with a checker
    # `groundplane` material, plus a robot-sized statistic/extent. Reuse that
    # floor — do NOT add a second plane, two coplanar planes at z=0 z-fight into
    # a speckled mess. Instead:
    #   - enlarge stat.extent so the camera near/far clipping covers the whole
    #     warehouse (with the default extent=0.8 a top-down view clips it away);
    #   - drop the floor reflectance (mirror glare under bright light).
    spec.stat.extent = 18.0
    spec.stat.center = [0.0, 0.0, 1.0]
    try:
        spec.material('groundplane').reflectance = 0.0
    except Exception:
        pass

    # Even, glare-free lighting. The MuJoCo default light is a SPOT light, which
    # over the world origin creates a bright hotspot ("abbaglio"); and the G1's
    # headlight uses specular=0.9. Use a DIRECTIONAL light (parallel rays → no
    # hotspot) and kill all specular highlights (light + viewer headlight) so
    # light-coloured surfaces don't blow out to white when viewed from above.
    sun = wb.add_light(pos=[0, 0, 15], dir=[-0.3, -0.4, -1.0])
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    sun.diffuse = [0.5, 0.5, 0.5]
    sun.specular = [0.0, 0.0, 0.0]
    sun.castshadow = 1
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.4, 0.4, 0.4]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]

    # World obstacles in LIDAR_GROUP
    winfo = world_info(world)
    for ge in winfo["geoms"]():
        if ge["shape"] == "box":
            gg = wb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=ge["size"],
                             pos=ge["pos"], rgba=ge["rgba"], quat=_yaw_quat(ge["yaw"]))
        else:
            gg = wb.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=ge["size"],
                             pos=ge["pos"], rgba=ge["rgba"])
        _grp = ge.get("group")
        gg.group = LIDAR_GROUP if _grp is None else int(_grp)
        gg.contype = 0      # no collision (kinematic robot) — visual + ray only
        gg.conaffinity = 0

    # Dynamic people (mocap bodies, moved by mujoco_sim)
    default_colors = [[0.85, 0.15, 0.15], [0.95, 0.55, 0.1], [0.6, 0.2, 0.7]]
    colors = people_colors or default_colors
    person_names = []
    for i in range(int(n_people)):
        c = colors[i % len(colors)]
        person_names.append(_add_person(wb, i, [c[0], c[1], c[2], 1.0]))

    # LiDAR mount site on torso_link
    spec.body('torso_link').add_site(
        name='mid360', pos=list(MID360_POS),
        quat=[math.cos(MID360_PITCH / 2), 0.0, math.sin(MID360_PITCH / 2), 0.0])

    model = spec.compile()

    # actuated (non-free) joints → for /joint_states
    free_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base_joint')
    joint_names, qpos_adr = [], []
    for j in range(model.njnt):
        if j == free_jid:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name is None:
            continue
        joint_names.append(name)
        qpos_adr.append(int(model.jnt_qposadr[j]))

    # mocap indices for the people (data.mocap_pos is indexed by body_mocapid)
    person_mocap_ids = []
    for nm in person_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, nm)
        person_mocap_ids.append(int(model.body_mocapid[bid]))

    info = dict(
        site_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'mid360'),
        free_qpos_adr=int(model.jnt_qposadr[free_jid]),
        joint_names=joint_names,
        joint_qpos_adr=qpos_adr,
        lidar_group=LIDAR_GROUP,
        person_mocap_ids=person_mocap_ids,
        world=world,
        world_spawn=tuple(winfo["spawn"]),
        world_goal=tuple(winfo["goal"]),
        world_desc=winfo["desc"],
    )
    return model, info
