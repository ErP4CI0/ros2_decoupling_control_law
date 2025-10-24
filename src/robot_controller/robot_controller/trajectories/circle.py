import numpy as np
import math


class Circle:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):
        self.velocity = 0.08 #Velocità tangenziale desiderata lungo la circonferenza del punto anteriore [m/s] (in Turtlesim ho messo 0.4)
        self.r = 0.4 #Raggio del cerchio [m] (in Turtlesim ho messo 2.0)
        self.w = self.velocity / self.r #Velocità angolare del moto circolare uniforme
        self.period = (2.0 * math.pi) / self.w

        #Array tempi discreti
        self.time_steps = np.arange(start=0., 
                                    stop=self.period, #almeno non ripetiamo il primo punto
                                    step=1.0 / hz)

        #Cerchio centrato in (0,-r) (frame locale)
        #Posizione (x(t),y(t)) parametrizzazione di un cerchio
        self.x_coord = self.r * np.sin(self.w * self.time_steps)
        self.y_coord = self.r * np.cos(self.w * self.time_steps) - self.r
        #Velocità (x'(t),y'(t))
        self.x_coord_dot = self.w * self.r * np.cos(self.w * self.time_steps)
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