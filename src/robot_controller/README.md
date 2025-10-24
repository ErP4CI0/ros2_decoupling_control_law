# Nodo `create_trajectory`

## Descrizione generale
Il nodo `create_trajectory` è responsabile della generazione di traiettorie di riferimento per il robot mobile TurtleBot Burger.  
Le traiettorie possono essere **parametriche** (retta, cerchio, sinusoide, otto, quadrato) oppure definite tramite **waypoint** forniti da un service dedicato (`waypoint_server`).

---

## Funzionalità principali
- Richiede i waypoint al service `/get_waypoints`
- Genera diverse tipologie di traiettorie (parametriche o spezzate)
- Pubblica i riferimenti su `/reference_trajectory`
- Gestisce il timeout e i casi di errore in modo sicuro
- Permette la rigenerazione di una nuova traiettoria a runtime

---

## Topic e Service

### Topic
| Nome | Tipo | Direzione | Descrizione |
|------|------|------------|--------------|
| `/reference_trajectory` | `robot_interfaces/msg/Trajectory` | Pubblicato | Traiettoria desiderata per il controllore |
| `/odom` | `nav_msgs/msg/Odometry` | Sottoscritto | Odometria del robot per allineamento iniziale |

### Service
| Nome | Tipo | Ruolo | Descrizione |
|------|------|-------|--------------|
| `/get_waypoints` | `robot_interfaces/srv/GetWaypoints` | Client | Ottiene i waypoint dal nodo `waypoint_server` |

---

## Parametri ROS2

| Nome | Tipo | Default | Descrizione |
|------|------|----------|-------------|
| `hz` | `float` | `20.0` | Frequenza operativa del nodo |
| `front_point` | `float` | `0.10` | Distanza del punto anteriore rispetto al baricentro |

---

## Tipologie di traiettoria supportate
- `retta`
- `sinusoide`
- `cerchio`
- `otto`
- `quadrato`
- `spezzata`
- `smooth`

---

## Esecuzione del nodo

Avviare il nodo dopo che il server `waypoint_server` è in esecuzione:

```bash
ros2 run robot_controller create_trajectory
