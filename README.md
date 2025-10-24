# ROS 2 Workspace — Decoupling Control Law for TurtleBot3 Burger

## Descrizione generale

Questo workspace ROS 2 implementa un sistema completo per il **controllo cinematico disaccoppiante** di un **TurtleBot3 Burger**.
L’obiettivo è permettere al robot di **seguire traiettorie arbitrarie** definite da waypoint o funzioni parametriche.

Il sistema è strutturato in più nodi ROS 2, ognuno con un compito ben definito, e fa uso di un package di interfaccia dedicato per lo scambio di messaggi e servizi.

---

## Architettura generale del sistema

- **`waypoint_server`** → fornisce i waypoint tramite un *ROS Service* ;
- **`create_trajectory`** → genera la traiettoria di riferimento ;
- **`controller_node_turtlebot`** → riceve la traiettoria e calcola i comandi di velocità da applicare al robot.

---

## Package inclusi

### `robot_interfaces`
Contiene la definizione dei messaggi e servizi comuni:
- `Trajectory.msg` → usato per trasmettere la traiettoria tra nodi;
- `GetWaypoints.srv` → usato dal client `create_trajectory` per richiedere i waypoint al server.

---

### `robot_controller`

Contiene tutti i nodi logici e funzionali del sistema:
- `waypoint_server.py` → fornisce i waypoint al client tramite servizio;
- `create_trajectory.py` → genera traiettorie parametriche o basate su waypoint;
- `isc_control_law_decoupling.py` → implementa la legge di controllo disaccoppiante per il TurtleBot.

All’interno del package si trovano anche:
- la cartella `trajectories/`, che contiene le diverse tipologie di traiettorie (retta, cerchio, otto, sinusoide, spezzata, smooth);
- la cartella `utils/`, con funzioni di supporto (es. conversione di quaternioni e lettura di messaggi);
- il file `config/waypoints.yaml`, dove vengono definiti i waypoint.
