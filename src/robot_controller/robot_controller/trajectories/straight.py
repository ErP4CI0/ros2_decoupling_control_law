import numpy as np


class Straight:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):
        self.velocity = 0.08 #Velocità [m/s] (costante) (in Turtlesim = 0.4)
        self.distance = 2.0 #Distanza da percorrere [m] (in Turtlesim = 4.0)
        #Array che indica l'avanzamento, rispetto al frame locale, del punto anteriore.
        #Indica l'evoluzione della posizione x(t),y(t). In questo caso ci moviamo
        #solamente lungo l'asse x locale
        self.trajectory = np.arange(start=0.0,
                                    stop=self.distance + (self.velocity / hz), #in modo tale da includere anche l'ultimo punto
                                    step=self.velocity / hz, # spazio = velocità * tempo 
                                    dtype=float)
        
        #Parametri per la conversione al frame globale
        self.rotation_matrix = rotation_matrix
        self.init_offset = init_offset
        
        #(x(t),y(t)) (global frame)
        self.ref_position = []
        #(x^.(t),y^.(t)) (global frame)
        self.ref_velocities = []
    
    #Questo perchè la posizione e velocità la calcoliamo rispetto al frame locale
    #e vogliamo "convertirla" nel frame globale (quello del robot fisico).
    def to_global_frame(self):
        for i in range(self.trajectory.size):
            #Punto 2D (x,y) (local frame)
            point_local = np.array([self.trajectory[i], 0.], dtype=float).T #Questo .T serve a traslare l'array, e quindi da un vettore riga
                                                                      #diviene un vettore colonna in modo tale da poter effettuare poi la 
                                                                      #moltiplicazione con la matrice di rotazione. In realtà python gestisce
                                                                      #tutti gli array 1D come vettori riga e quindi ciò non comporta nessun
                                                                      #cambiamento. Quindi in realtà non serve a nulla ma concettualemnte ci 
                                                                      #permette di capire ciò.
            #Velocità (x^.,y^.) in quell'istante. Abbiamo detto essere costante e lungo l'asse x locale, quindi:
            point_dot_local = np.array([self.velocity, 0.],dtype=float).T
            #Adattiamo rispetto al frame globale sfruttando la matrice di rotazione
            point_global = np.dot(self.rotation_matrix, point_local).reshape((2,)) + self.init_offset #Anche qua .dot() resituisce un'array (2,), lo 
                                                                                                      #si mette per robustezza (in caso restituisse (2,1))
            point_dot_global = np.dot(self.rotation_matrix, point_dot_local).reshape((2,))

            #Aggiungiamo alle liste
            self.ref_position.append(point_global)
            self.ref_velocities.append(point_dot_global)
        
        return self.ref_position, self.ref_velocities
        
