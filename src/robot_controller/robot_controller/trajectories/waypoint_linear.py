import math
import numpy as np


class WaypointLinear:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray, waypoints: list):

        self.way_points = waypoints
        
        self.velocity = 0.07 #Velocità desiderata lungo il percorso
        self.hz = hz #Frequenza di lavoro

        #Parametri per la conversione al frame globale
        self.rotation_matrix = rotation_matrix
        self.init_offset = init_offset

        #Array contenenti posizioni e velocità finali dell'intera traiettoria (in locale)
        self.x_coord = np.array([], dtype=float)
        self.y_coord = np.array([], dtype=float)
        self.x_coord_velocity = np.array([], dtype=float)
        self.y_coord_velocity = np.array([], dtype=float)

        #Posizione e velocità (global frame, da passare al nodo controllore)
        self.ref_position = []
        self.ref_velocities = []

        self.compute_trajectory()
    
    def compute_trajectory(self):
        #Il numero di rette è pari al numero di waypoint - 1
        for i in range(len(self.way_points) - 1):
            first_index_waypoint = i
            second_index_waypoint = i + 1
            #Distanza tra due waypoint, lunghezza vettore |P0 P1| (alla prima iterazione), |P1 P2| alla seconda..
            distance_way_points = math.sqrt((self.way_points[second_index_waypoint][0] - self.way_points[first_index_waypoint][0]) ** 2 
                                        + (self.way_points[second_index_waypoint][1] - self.way_points[first_index_waypoint][1]) ** 2)
            #OSS: per robustezza potremmo verificare il fatto che non sia nullo come valore (=sovrapposizione waypoint)

            #Tempo di percorrenza della distanza
            period = distance_way_points / self.velocity
         
            #Array tempi discreti 
            time_steps = np.arange(start=0.0, step=1.0 / self.hz , stop=period + (1.0 / self.hz)) #Così includo anche l'ultimo valore se è preciso,
                                                                                              #altrimenti possono esserci delle imprecisioni:
                                                                                              #l'ultimo valore può non essere esatto, non coincidere
                                                                                              #potremmo forzarlo.
            time_steps[-1] = period

            #Distanza da percorre ad ogni istante di tempo lungo la traiettoria
            space_steps = time_steps * self.velocity

            #Coordinate realtive alla posizione di un segmento. (Vedi ragionamento in fondo alla pagina (relativo ad un segmento))
            x_coord_segment = self.way_points[first_index_waypoint][0] + (space_steps / distance_way_points) * (self.way_points[second_index_waypoint][0] - self.way_points[first_index_waypoint][0])
            y_coord_segment = self.way_points[first_index_waypoint][1] + (space_steps / distance_way_points) * (self.way_points[second_index_waypoint][1] - self.way_points[first_index_waypoint][1])
            
            #Nei tratti successivi al primo, abbiamo una ripetizione del primo punto, e quindi eliminiamo il primo elemento
            if first_index_waypoint > 0: #dopo il primo
                x_coord_segment = np.delete(x_coord_segment, 0)
                y_coord_segment = np.delete(y_coord_segment, 0)

            #Concateniamo (con molti punti potrebbe non essere efficiente in quanto si crea un nuovo array ogni volta)
            self.x_coord = np.concatenate((self.x_coord, x_coord_segment))
            self.y_coord = np.concatenate((self.y_coord, y_coord_segment))

            #Vedi ragionamento (sono valori scalari in quanto costanti)
            x_coord_velocity_segment = self.velocity * ((self.way_points[second_index_waypoint][0] - self.way_points[first_index_waypoint][0]) / distance_way_points) #vx lungo la traiettoria
            y_coord_velocity_segment = self.velocity * ((self.way_points[second_index_waypoint][1] - self.way_points[first_index_waypoint][1]) / distance_way_points) #vy lungo la traiettoria
            
            dim_array = len(x_coord_segment)
            x_coord_velocity_segment_array = np.full(dim_array, x_coord_velocity_segment) #creo un array con lo stesso valore essendo uno scalare
            y_coord_velocity_segment_array = np.full(dim_array, y_coord_velocity_segment)
            #Concateniamo
            self.x_coord_velocity = np.concatenate((self.x_coord_velocity, x_coord_velocity_segment_array))
            self.y_coord_velocity = np.concatenate((self.y_coord_velocity, y_coord_velocity_segment_array))

    
    def to_global_frame(self):
        for i in range(self.x_coord.size):
            #Punto (posizione) 2D (x,y) (local frame)
            point_local = np.array([self.x_coord[i], self.y_coord[i]], dtype=float).T 
            #(Velocità)
            point_dot_local = np.array([self.x_coord_velocity[i], self.y_coord_velocity[i]], dtype=float).T
            #Adattiamo rispetto al frame globale sfruttando la matrice di rotazione
            point_global = np.dot(self.rotation_matrix, point_local).reshape((2,)) + self.init_offset
            point_dot_global = np.dot(self.rotation_matrix, point_dot_local).reshape((2,))
            #Aggiungiamo alle liste
            self.ref_position.append(point_global)
            self.ref_velocities.append(point_dot_global)
        
        return self.ref_position, self.ref_velocities



#RAGIONAMENTO COORDINATE:
#Una retta che passa per due punti (A=(x1,y1) e B=(x2,y2))può essere scritta in forma parametrica così:
#(x,y)=(x1,y1)+t(x2-x1,y2-y1).
#Qui t è un parametro che indica quanto cammino lungo la retta. Se t=0 sono in A,
#se t=1 sono in B..
#La lunghezza del vettore AB è |AB| = √(x2-x1)^2 + (y2-y1)^2.
#Se il vettore AB ha lunghezza |AB| allora il vettore t AB ha una lunghezza che dipende da t,
#e quindi ne deduciamo che s = t * |AB| (s=distanza effettivamente percorsa), e quindi t=s/|AB|,
#cioè per percorre lo spazio s occore moltiplicare |AB| per un valore pari a t.
#Nel nostro caso abbiamo una distanza fissa (d), e quindi t=d/|AB|.
#Sostituendo nella formula iniziale otteniamo che:
#x = x1 + (d/|AB|) * (x2-x1)
#y = y1 + (d/|AB|) * (y2-y1)
#Spieghiamo invece ora il ragionamento per quanto rigurda il calcolo delle velocità.
#Quello che vogliamo è muoversi lungo la direzione della retta che passa per quei due 
#waypoint, e vogliamo muoverci con velocità costante v. E vogliamo trovare quindi le
#componenti (vx,vy) tali che la direzione è quella di A-->B e che √(vx)^2 + (vy)^2 = v.
#La direzione è data dal vettore AB = (x2-x1, y2-y1), e la sua lunghezza è |AB|.
#Per ottenere la sola direzione creiamo il versore: u=((x2-x1) / |AB|, (y2-y1) / |AB|).
#La velocità deve puntare nella stessa direzione di u (è un versore), ma con lunghezza v(scalare).
#Quindi il vettore v (=(vx,vy)) deve essere uguale a: v = v(scalare) * u(versore). Quindi:
#vx = v * (x2-x1)/|AB|
#vy = v * (y2-y1)/|AB|
#Qua sotto inserisco il codice relativo al primo tratto retta che avevo fatto (caso base), il quale poi ho modificato
#class WaypointLinear:
#    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):

#        self.way_points = [[0.0, 0.0],  # P1
#                           [2.0, 0.0],  # P2
#                           [3.0, 2.0],  # P3
#                           [4.0, 2.5]]  # P4
        
#        self.velocity = 0.08  # Velocità desiderata lungo il percorso
#        self.hz = hz  # Frequenza di lavoro

        # Parametri per la conversione al frame globale
#        self.rotation_matrix = rotation_matrix
#        self.init_offset = init_offset

        # Posizione e velocità (global frame)
#        self.ref_position = []
#        self.ref_velocities = []

#        self.compute_trajectory()
    
#    def compute_trajectory(self):
        # Distanza tra due waypoint, lunghezza vettore |P1 P2|
#        distance_way_points = math.sqrt(
#            (self.way_points[1][0] - self.way_points[0][0]) ** 2 +
#            (self.way_points[1][1] - self.way_points[0][1]) ** 2)

        # Tempo di percorrenza della distanza
#        period = distance_way_points / self.velocity
         
        # Array tempi discreti 
#        time_steps = np.arange(start=0.0, step=1.0 / self.hz, stop=period + (1.0 / self.hz))
#        time_steps[-1] = period

        # Distanza da percorrere ad ogni istante di tempo lungo la traiettoria
#        space_steps = time_steps * self.velocity

        # Coordinate (vedi ragionamento in fondo)
#        self.x_coord = self.way_points[0][0] + (space_steps / distance_way_points) * (self.way_points[1][0] - self.way_points[0][0])
#        self.y_coord = self.way_points[0][1] + (space_steps / distance_way_points) * (self.way_points[1][1] - self.way_points[0][1])

        # Componenti di velocità lungo gli assi
#        self.x_coord_velocity = self.velocity * ((self.way_points[1][0] - self.way_points[0][0]) / distance_way_points)
#        self.y_coord_velocity = self.velocity * ((self.way_points[1][1] - self.way_points[0][1]) / distance_way_points)
    
#    def to_global_frame(self):
#        for i in range(self.x_coord.size):
            # Punto (posizione) 2D (local frame)
#            point_local = np.array([self.x_coord[i], self.y_coord[i]], dtype=float).T
            # Velocità
#            point_dot_local = np.array([self.x_coord_velocity, self.y_coord_velocity], dtype=float).T
            # Conversione al frame globale
#            point_global = np.dot(self.rotation_matrix, point_local).reshape((2,)) + self.init_offset
#            point_dot_global = np.dot(self.rotation_matrix, point_dot_local).reshape((2,))
            # Salvataggio
#            self.ref_position.append(point_global)
#            self.ref_velocities.append(point_dot_global)
        
#        return self.ref_position, self.ref_velocities


