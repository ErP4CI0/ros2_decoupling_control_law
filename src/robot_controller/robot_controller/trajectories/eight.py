import numpy as np
import math


class Eight:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):
        self.velocity = 0.08 
        self.r = 0.4 
        self.w = self.velocity / self.r 
        self.period = (2.0 * math.pi) / self.w

        #Array tempi discreti
        self.time_steps = np.arange(start=0., 
                                    stop=self.period, #almeno non ripetiamo il primo punto
                                    step=1.0 / hz)

        #Posizione (x(t),y(t)) parametrizzazione di un cerchio
        self.x_coord = self.r * np.sin(2.0 * self.w * self.time_steps)
        self.y_coord = self.r * np.cos(self.w * self.time_steps) - self.r
        #Velocità (x'(t),y'(t))
        self.x_coord_dot = 2.0 * self.w * self.r * np.cos(2.0 *self.w * self.time_steps)
        self.y_coord_dot = -self.w * self.r * np.sin(self.w * self.time_steps)

        #Parametri per la conversione al frame globale
        self.rotation_matrix = rotation_matrix
        self.init_offset = init_offset
        
        #(x(t),y(t)) (global frame)
        self.ref_position = []
        #(x^.(t),y^.(t)) (global frame)
        self.ref_velocities = []
    
    
    def to_global_frame(self):
        for i in range(self.time_steps.size):
            #Punto 2D (x,y) (local frame)
            point_local = np.array([self.x_coord[i], self.y_coord[i]], dtype=float).T 
            point_dot_local = np.array([self.x_coord_dot[i], self.y_coord_dot[i]], dtype=float).T
            #Adattiamo rispetto al frame globale sfruttando la matrice di rotazione
            point_global = np.dot(self.rotation_matrix, point_local).reshape((2,)) + self.init_offset
            point_dot_global = np.dot(self.rotation_matrix, point_dot_local).reshape((2,))
            #Aggiungiamo alle liste
            self.ref_position.append(point_global)
            self.ref_velocities.append(point_dot_global)
        
        return self.ref_position, self.ref_velocities
#SPIEGAZIONE APPROFONDITA:
#La forma dell'otto deriva da una combinazione di due moti armonici con frequenze diverse, cioè un rapporto di 2:1 
#tra le componenti lungo gli assi X e Y:
#x(t) = r * sin(2 * ω * t)
#y(t) = r * cos(ω * t) - r
#In questo modo, mentre l’asse Y completa un solo ciclo (un giro), l’asse X ne completa due.  
#Il risultato è che il punto attraversa due volte l’origine (il centro della figura) in direzioni opposte, 
#disegnando due lobi simmetrici, che formano una curva chiusa a forma di 8.
#Il termine −r nella coordinata Y serve solo a traslare la figura verso il basso, 
#in modo che la traiettoria parta da un punto più realistico (di solito davanti al robot).