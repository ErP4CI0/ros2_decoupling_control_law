# Nodo `waypoint_server`

## Descrizione generale

Il nodo **`waypoint_server`** è un nodo sviluppato in **ROS 2** che fornisce, tramite un **ROS Service**, una lista di **waypoint** (punti di riferimento nello spazio) ad altri nodi che ne fanno richiesta, in particolare al nodo `create_trajectory`.

---

## Funzionamento

All’avvio del nodo vengono eseguiti i seguenti passaggi principali:

1. **Lettura del file YAML**
   - Il nodo ricerca un file denominato `waypoints.yaml`, contenente le coordinate dei punti da inviare.
   - Il file deve rispettare la seguente struttura:

     ```yaml
     waypoints:
       - [0.0, 0.0]
       - [2.0, 0.0]
       - [3.0, 2.0]
       - [4.0, 2.5]
     ```

   - Ogni riga rappresenta un punto nello spazio identificato dalle coordinate `(x, y)`.

2. **Creazione del service `/get_waypoints`**
   - Il nodo pubblica un **service** denominato `/get_waypoints`, definito nel file:
     ```
     robot_interfaces/srv/GetWaypoints.srv
     ```
     (per i dettagli sulla struttura, si veda il file `.srv` all’interno del package `robot_interfaces`).

   - Quando un client, come ad esempio `create_trajectory`, invia una richiesta, il server:
     - legge i waypoint dal file YAML;
     - li converte in messaggi `geometry_msgs/Point`;
     - restituisce la lista dei waypoint al client, includendo un flag di successo (`success`) e un messaggio descrittivo (`message`) che riporta lo stato dell’operazione o il numero di waypoint trasmessi.

3. **Gestione degli errori**
   - Se il file YAML non esiste, non è leggibile oppure risulta malformato, il nodo entra in **modalità interattiva**: l’utente viene invitato a inserire manualmente i waypoint tramite terminale.

---

## Posizione corretta di esecuzione

Il nodo `waypoint_server` deve essere eseguito **dal livello corretto del workspace**, poiché per impostazione predefinita ricerca il file di configurazione nel percorso relativo:
```
config/waypoints.yaml
```

Se il nodo viene lanciato da una posizione diversa o se si desidera utilizzare un file alternativo, è possibile **specificare esplicitamente il percorso del file YAML** tramite parametro ROS 2 al momento dell’esecuzione:

```bash
ros2 run robot_controller waypoint_server --ros-args -p file_path:=/home/giovanni/ros2_decoupling_control_law/src/robot_controller/robot_controller/config/waypoints.yaml
```

# Nodo `create_trajectory`

## Descrizione generale

Il nodo **`create_trajectory`** è responsabile della **generazione della traiettoria di riferimento** da fornire al controllore del TurtleBot Burger.
Rappresenta il componente di collegamento tra la fase di **definizione dei waypoint** (gestita dal nodo `waypoint_server`) e la fase di **controllo del movimento**.

La sua funzione principale è quella di:
1. richiedere i waypoint al service `/get_waypoints`;
2. generare una traiettoria di riferimento in base al tipo selezionato;
3. pubblicarla sul topic `/reference_trajectory` per renderla disponibile al controllore.

---

## Funzionamento del nodo

All’avvio, il nodo esegue in sequenza i seguenti passaggi principali:

### 1. **Inizializzazione dei parametri**
- Crea un publisher sul topic `/reference_trajectory`, di tipo `robot_interfaces/msg/Trajectory`(visionare file).
- Imposta la frequenza di lavoro (`hz = 20.0 Hz`) e il punto di controllo anteriore (`front_point = 0.10 m`) (tali valori devono essere corenti con quelli del nodo controllore).
- Inizializza la matrice di rotazione, che servirà per la conversione tra frame locali e globali.

### 2. **Connessione al Service `get_waypoints`**
Il nodo si comporta da **client** del service `/get_waypoints`, implementato dal nodo `waypoint_server`.
La sequenza di operazioni è la seguente:

1. Creazione del client:
   ```python
   self.client = self.create_client(GetWaypoints, 'get_waypoints')
    ```
   Viene creato un client ROS2 che si collegherà al service /get_waypoints del server.

2. Attesa della disponibilità del server:
    ```python
    self.wait_for_service_availability(timeout_sec=20.0)
    ```
    Il nodo resta in attesa per un massimo di 20 secondi. Se il server non risponde il nodo viene interrotto.

3. Invio della richiesta al server:
    Una volta che il serivice è disponibile, viene inviata la richiesta in modo asincron:
    ```python
    future = self.client.call_async(req)
    ```
    (Evita che il programma si blocchi completamente durante l'attesa)

4. Attesa della risposta con timeout:
    ```python
    rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
    ```
    Il nod rimane in attesa della risposta per un massimo di 5 secondi.
    Se il tempo scade vengono usati **waypoint di default**

5. Verifica della risposta ricevuta:
    Se la risposta è valida, viene estratta la lista dei waypoint da utilizzare per la generazione della traiettoria.

---

## Selezione del tipo di traiettoria

L'utente sceglie da terminale il tipo di traiettoria da generare, tra le seguenti opzioni:
`[retta, sinusoide, cerchio, otto, quadrato, spezzata, smooth]`
A seconda della scelta viene istanziata una classe corrispondente presente nella cartella `robot_controller/trajectories`

## Tipologie di traiettoria: *spezzata* e *smooth*

Il progetto si concentra in particolare sulle traiettorie **spezzata (lineare)** e **smooth (smussata)**, che rappresentano due modalità diverse di interpolazione dei waypoint forniti dal nodo `waypoint_server`.

### Traiettoria *spezzata* (lineare)

La traiettoria **spezzata** collega i waypoint mediante segmenti di retta.
Ogni tratto tra due waypoint consecutivi è percorso in modo lineare, senza continuità di direzione tra un segmento e l’altro.
Questo tipo di traiettoria si basa su una **interpolazione lineare** dei punti, mantenendo una direzione costante fino al raggiungimento del waypoint successivo.

---

### Traiettoria *smooth* (curva)

La traiettoria **smooth** mira invece a generare un percorso **continuo e regolare**, evitando cambi di direzione bruschi.
Per ottenere questa proprietà di *continuità in derivata prima* (cioè senza discontinuità nella velocità), viene utilizzata una **interpolazione tramite curve di Bézier**.

Ogni tratto tra due waypoint è approssimato da una curva di Bézier di grado 3, che garantisce una transizione morbida tra le diverse direzioni.

## Inizializzazione della posa iniziale

Prima di generare la traiettoria, il nodo acquisisce la posa corrente del robot leggendo un messaggio dal topic `/odom`.
Questo passaggio consente di allineare la traiettoria al frame globale del robot.

In particolare il nodo:

- legge il quaternione (orientation) dal messaggio Odometry;

- ne calcola lo yaw mediante la funzione yaw_from_quaternion (definita in `utils/utility.py`);

- calcola la posizione del punto anteriore del robot e costruisce la matrice di rotazione che servirà per proiettare la traiettoria nel sistema di riferimento globale.

## Pubblicazione della traiettoria

Una volta calcolate le posizioni e le velocità, il nodo costruisce un messaggio ROS di tipo Trajectory e lo pubblica sul topic /reference_trajectory.
 ```python
traj_msg = Trajectory()
for pos, vel in zip(self.ref_position, self.ref_velocities):
    traj_msg.positions.append(Point(x=pos[0], y=pos[1], z=0.0))
    traj_msg.velocities.append(Vector3(x=vel[0], y=vel[1], z=0.0))
self.trajectory_pub.publish(traj_msg)
 ```

Il controllore del robot, sottoscritto a questo topic, potrà così ricevere i riferimenti da seguire.


# Nodo `controller_node_turtlebot (isc_control_law_decoupling)`

## Descrizione generale

Il nodo **`controller_node_turtlebot`** implementa la **legge di controllo disaccoppiante** per il TurtleBot Burger. 
Il suo scopo è ricevere la traiettoria di riferimento generata dal nodo `create_trajectory`, confrontarla con la posa reale del robot (ricevuta tramite odometria) e calcolare i comandi di velocità lineare e angolare da inviare al robot.

In sintesi, realizza l’anello di controllo che consente al robot di **seguire una traiettoria predefinita** nel piano.

---

## Funzionamento del nodo

All’avvio, il nodo esegue le seguenti operazioni principali:

### 1. **Inizializzazione dei parametri**

- Definisce i **parametri di controllo**:
  - `front_point`: distanza del punto di controllo anteriore (0.10 m);
  - `kx`, `ky`: guadagni proporzionali per il controllo in x e y.
- Inizializza lo **stato del robot** (`current_pose`) come vettore `[x, y, θ]`.
- Imposta la **frequenza di lavoro** a 20 Hz.
- Prepara le liste per:
  - i riferimenti di traiettoria (`ref_position`, `ref_velocities`);
  - i valori misurati, utili per il calcolo dell’errore quadratico medio (`rmse_x_real`, `rmse_y_real`).

---

### 2. **Publisher e Subscriber**

Il nodo utilizza diversi **publisher** e **subscriber** per comunicare con il resto del sistema ROS 2:

#### Publisher principali:
- `/cmd_vel` → invio dei comandi di velocità (`Twist`) al robot;

- `/ref_path` e `/real_path` → pubblicazione dei percorsi completi (`nav_msgs/Path`), utili per visualizzare in **RViz2** la traiettoria seguita e quella di riferimento.

#### Subscriber:
- `/odom` → riceve messaggi di tipo `Odometry` per aggiornare la posa corrente del robot;
- `/reference_trajectory` → riceve la traiettoria di riferimento (`Trajectory`) generata da `create_trajectory`.

---

### 3. **Aggiornamento della posa**

La funzione `pose_updater()` viene chiamata ad ogni nuovo messaggio ricevuto dal topic `/odom`. 
Aggiorna lo stato interno del robot (`current_pose`) estraendo:
- la posizione `(x, y)` del centro del robot;
- l’orientamento `θ`, calcolato dal quaternione tramite la funzione `yaw_from_quaternion()`.

---

### 4. **Ricezione della traiettoria**

Quando arriva un nuovo messaggio sul topic `/reference_trajectory`, viene eseguita la `trajectory_callback()`, che:
- interrompe la traiettoria precedente (ferma il robot);
- svuota le liste di riferimento;
- salva la nuova lista di posizioni e velocità da seguire.

In questo modo il nodo è sempre pronto a gestire nuove traiettorie in tempo reale.

---

### 5. **Legge di controllo disaccoppiante**

Il cuore del nodo è la funzione `control()`, che implementa una **legge di controllo disaccoppiante** per i modelli cinematici differenziali.

#### Passaggi principali:
1. Si considera come punto da controllare il **punto anteriore** del robot (a distanza `front_point` dal centro).
2. Si costruisce la **matrice di accoppiamento**:

3. Si calcola la sua **inversa** , ottenendo una legge che disaccoppia i due comandi `(v, w)`.

4. Si applica un **controllo proporzionale** sui due errori lungo gli assi.

5. Si individuano i comandi (v, w) necessari per l'inseguimento.

Questa strategia garantisce che il robot converga verso il riferimento desiderato in modo stabile e indipendente dalle dinamiche di accoppiamento.

---

### 6. **Ciclo di controllo (`control_loop`)**

Il timer del nodo richiama periodicamente (20 Hz) la funzione `control_loop()`, che:

1. verifica se la traiettoria è disponibile;
2. preleva il prossimo punto di riferimento da analizzare;
3. calcola (v,w) tramite la funzione di controllo;
4. pubblica il comando di velocità sul topic `/cmd_vel`;
5. aggiorna/pubblica sui vari topic
6. incrementa l’indice `traj_index` per passare al punto successivo da analizzare.

Quando tutti i punti della traiettoria sono stati eseguiti:
- viene calcolata la metrica **RMSE** (Root Mean Square Error);
- il robot viene arrestato;
- le variabili interne vengono resettate...

---

### 7. **Calcolo della metrica RMSE**

Alla fine della traiettoria, la funzione `calculate_rmse()` valuta la precisione del tracciamento mediante la metrica RMSE.

Il valore di RMSE fornisce una misura quantitativa dell’errore medio commesso nel seguire la traiettoria.

---

## 8. **Visualizzazione in RViz2**

Il nodo pubblica sia i **punti singoli** (`/ref_signal`, `/xb_signal`) sia i **percorsi completi** (`/ref_path`, `/real_path`), rendendo possibile una visualizzazione in RViz2 del tracciamento

---

## 9. **Terminazione sicura**

Quando la traiettoria è completata o il nodo viene interrotto manualmente (`Ctrl + C`):
- il metodo `stop_node()` invia dieci messaggi `Twist()` nulli (per garantire che il robot si arresti fisicamente);
- il nodo viene distrutto e lo shutdown di ROS 2 viene eseguito in modo ordinato.

---

## Considerazioni finali

Il nodo `controller_node_turtlebot` realizza un controllo cinematico **disaccoppiante e proporzionale**, capace di seguire traiettorie arbitrarie in modo stabile e preciso.
L’uso dei topic `Path` e `PointStamped` permette inoltre un **monitoraggio visivo** del comportamento del robot in RViz2, facilitando la validazione sperimentale del sistema.

La struttura modulare del nodo consente infine di:
- riutilizzare facilmente la logica di controllo con altri tipi di traiettoria;
- adattare i guadagni (`kx`, `ky`) a differenti dinamiche robotiche;
- integrare ulteriori controllori o strategie di correzione in futuro.

---

# Avvio e interazione tra i nodi

Il sistema ROS 2 sviluppato è composto da tre nodi principali che lavorano in sequenza per consentire al robot TurtleBot Burger di seguire una traiettoria desiderata:

**controller_node_turtlebot ← create_trajectory ← waypoint_server**


In altre parole:
1. il **controllore** riceve la traiettoria di riferimento e comanda il robot;
2. il **create_trajectory** genera tale traiettoria in base ai waypoint ricevuti;
3. il **waypoint_server** fornisce i waypoint richiesti tramite un servizio ROS.

---

## Ordine corretto di esecuzione

Per garantire un corretto funzionamento del sistema, i nodi devono essere avviati **in questo ordine**:

### Controller node
È il primo nodo da lanciare, perché:
- sottoscrive il topic `/reference_trajectory` (necessario per ricevere la traiettoria);
- pubblica i comandi `/cmd_vel` che muovono il robot.

### Waypoint server
È il secondo nodo da avviare in qaunto fornisce i waypoint tramite il servizio `/get_waypoints`, richiesto successivamente dal nodo `create_trajectory`.

**OSS**: come detto in precedenza assicurati di eseguirlo nella posizione corretta del workspace (/robot_controller/robot_controller/), oppure specificando il percorso del file YAML.

### Create trajectory
Ultimo nodo da avviare.
Agisce come client del waypoint_server, riceve i waypoint, genera la traiettoria desiderata e la pubblica su `/reference_trajectory`, topic a cui il controllore è già in ascolto. 



