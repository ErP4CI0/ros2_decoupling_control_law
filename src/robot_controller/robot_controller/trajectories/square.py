#IDEA:
#L'idea è quella di creare un quadrato di lato L che il punto anteriore del robot
#deve percorrere a velocità costante v lungo ogni lato, fermandosi idealmente per 
#girare gli spigoli. Il robot quindi (localmente) va dritto in X (da 0 a L), va 
#dritto in Y (da 0 a L), torna inidetro in X (da L a 0) e torna indietro in Y (va
#da L a 0).
import numpy as np
import math


class Square:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):
        self.velocity = 0.08 #Velocità lineare costante
        
        self.r = 0.4 #Raggio del cerchio 
        self.w = self.velocity / self.r #Velocità angolare del moto circolare uniforme
        self.period = (2.0 * math.pi) / self.w
        #Qua non stiamo ruotando, stiamo "riciclando" il periodo del moto circolare uniforme
        #per far sì che il tempo totale del quadrato sia paragonabile a quello del cerchio,
        #quindi è una convenzione temporale, serve per avere traiettorie di durata simile.

        self.t_4 = np.arange(start=0., stop=self.period / 4, step=1.0 / hz)
        #Dividiamo il periodo in 4 parti, una per ogni lato. Stiamo dicendo che ogni lato
        #dura quindi T/4 secondi

        #Creiamo degli array di supporto
        self.t_max = np.ones_like(self.t_4) * self.t_4[-1]
        self.t_min = np.zeros_like(self.t_4)
        #Questi due servono per riempire i tratti in cui una coordianta deve rimanere ferma,
        #t_max è un array di valori massimo costanti, lo usiamo per rappresentare una coordinata bloccata
        #al valore massimo del lato. Discorso opposto per t_min

        #Costruiamo ora due array dei tempi che descrivono tutto il percorso nel tempo:
        self.t_x = np.concatenate([self.t_4, #x cresce e y rimane femro
                                   self.t_max, #x rimane massimo (fermo) e y cresce ...
                                   np.flip(self.t_4),
                                   self.t_min])
        self.t_y = np.concatenate([self.t_min,
                                   self.t_4,
                                   self.t_max,
                                   np.flip(self.t_4)])
        
        #Convertiamo il tempo in spazio:
        self.square_x = self.t_x * self.velocity
        self.square_y = self.t_y * self.velocity
        #Ora questi due aray descrivono la forma del quadrato in metri

        #In Questa utlima parte indichiamo lungo quali lati occre dare velocità.
        #Lungo gli spigoli c'è un cambimento brusco di velocità, una componente si annulla subito
        #si ha una discontinuità sulla velocità
        self.non_null_vel = np.ones_like(self.t_4)
        self.null_vel = np.zeros_like(self.t_4)
        self.square_x_dot = np.concatenate([self.velocity * self.non_null_vel, self.null_vel, -self.velocity * self.non_null_vel, self.null_vel])
        self.square_y_dot = np.concatenate([self.null_vel, self.velocity * self.non_null_vel, self.null_vel, -self.velocity * self.non_null_vel])

        #Parametri per la conversione al frame globale
        self.rotation_matrix = rotation_matrix
        self.init_offset = init_offset
        
        #(x(t),y(t)) (global frame)
        self.ref_position = []
        #(x^.(t),y^.(t)) (global frame)
        self.ref_velocities = []
    
    
    def to_global_frame(self):
        for i in range(self.square_x.size):
            #Punto 2D (x,y) (local frame)
            point_local = np.array([self.square_x[i], self.square_y[i]], dtype=float).T 
            point_dot_local = np.array([self.square_x_dot[i], self.square_y_dot[i]], dtype=float).T
            #Adattiamo rispetto al frame globale sfruttando la matrice di rotazione
            point_global = np.dot(self.rotation_matrix, point_local).reshape((2,)) + self.init_offset
            point_dot_global = np.dot(self.rotation_matrix, point_dot_local).reshape((2,))
            #Aggiungiamo alle liste
            self.ref_position.append(point_global)
            self.ref_velocities.append(point_dot_global)
        
        return self.ref_position, self.ref_velocities
