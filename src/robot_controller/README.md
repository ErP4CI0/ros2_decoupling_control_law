# Nodo `waypoint_server`

## Descrizione generale

Il nodo **`waypoint_server`** è un nodo **ROS 2** scritto in Python che si occupa di fornire, tramite un **ROS Service**, una lista di **waypoint** (punti di riferimento nello spazio) ad altri nodi che ne hanno bisogno, in particolare al nodo `create_trajectory`.

Lo scopo è quello di **separare la logica di generazione delle traiettorie** dalla **definizione dei punti di passaggio**, migliorando la **modularità** del sistema.  
In pratica:
- il nodo `waypoint_server` gestisce i **dati** (i waypoint);
- il nodo `create_trajectory` si occupa di **generare le traiettorie** a partire da essi.

---

## ⚙️ Funzionamento

Quando il nodo viene avviato, esegue i seguenti passaggi:

1. **Lettura del file YAML**
   - Il nodo cerca un file `waypoints.yaml` che contiene le coordinate dei punti da inviare.
   - Il file deve avere questa struttura:

     ```yaml
     waypoints:
       - [0.0, 0.0]
       - [2.0, 0.0]
       - [3.0, 2.0]
       - [4.0, 2.5]
     ```

   - Ogni riga rappresenta un punto `(x, y)` nello spazio.

2. **Creazione del service `/get_waypoints`**
   - Il nodo pubblica un **service** denominato `/get_waypoints`, definito nel file:
     ```
     robot_interfaces/srv/GetWaypoints.srv
     ```
   - Quando un client (come `create_trajectory`) invia una richiesta, il server:
     - legge i waypoint dal file YAML;
     - li converte in messaggi ROS (`geometry_msgs/Point`);
     - restituisce la lista al client;
     - include anche un flag di successo (`success`) e un messaggio di stato (`message`).

3. **Gestione degli errori**
   - Se il file YAML non esiste, non è leggibile o non contiene waypoint validi:
     - `success = False`
     - `message = "Errore nella lettura del file YAML"`
   - In questo caso, il nodo client (ad esempio `create_trajectory`) può usare dei **waypoint di default**.

---

## 🧩 Definizione del Service

Il service è definito nel file:

```bash
robot_interfaces/srv/GetWaypoints.srv
