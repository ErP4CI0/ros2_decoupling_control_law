# Package `robot_interfaces`

## Descrizione generale

Il package **`robot_interfaces`** contiene la definizione dei **messaggi (`.msg`)** e dei **servizi (`.srv`)** necessari per la comunicazione tra i vari nodi del progetto:

- `waypoint_server`
- `create_trajectory`
- `controller_node_turtlebot`

Questo package fornisce quindi le **strutture dati comuni** che permettono ai nodi di scambiarsi informazioni in modo coerente e standardizzato.

---

---

## Messaggio: `Trajectory.msg`

### Descrizione

Il file **`Trajectory.msg`** definisce la struttura del messaggio che rappresenta una **traiettoria di riferimento**, pubblicata dal nodo `create_trajectory` e letta dal nodo `isc_control_law_decoupling`.

### Contenuto
```plaintext
geometry_msgs/Point[] positions
geometry_msgs/Vector3[] velocities
```

### Spiegazione dei campi
**`positions`** → array di punti (`geometry_msgs/Point`) che rappresentano le coordinate spaziali (x, y, z) del percorso da seguire (z = 0);

**`velocities`** → array di vettori (`geometry_msgs/Vector3`) che rappresentano le velocità lineari associate a ciascun punto della traiettoria.

---

## Service: `GetWaypoints.srv`

### Descrizione

Il file **`GetWaypoints.srv`** definisce la struttura del servizio utilizzato dal nodo `create_trajectory` per richiedere al nodo `waypoint_server` la lista dei waypoint da cui generare la traiettoria.

### Contenuto
# Richiesta
---

# Risposta
```plaintext
geometry_msgs/Point[] waypoints
bool success
string message
```


### Spiegazione dei campi

- **`Richiesta`**: vuota, poiché il client (`create_trajectory`) deve semplicemente richiedere i waypoint.

- **`Risposta`**:
    - **waypoints**: lista di punti contenenti le coordinate (x, y, z) dei waypoint letti dal file YAML o inseriti manualmente

    - **success**: valore booleano che indica se la richiesta è stata completata con successo;

    - **message**: stringa descrittiva che fornisce informazioni aggiuntive (es. “4 waypoint caricati con successo”).


