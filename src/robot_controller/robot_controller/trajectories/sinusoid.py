import numpy as np
import math


class Sinusoid:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray):
        self.velocity = 0.08 #Velocità [m/s], (in Turtlesim ho messo 0.4)
        self.distance = 3.0 #Distanza (utile per il calcolo di T)[m] (in Turtlesim ho messo 9.0)
        #Array tempi discreti
        self.time_steps = np.arange(start=0., 
                               stop=self.distance / self.velocity, #il tempo che ci mettero a percorre quella distanza a quella velocità
                               step=1.0 / hz, #ogni quanto campioniamo e analizziamo il segnale 
                               dtype=float)        
        self.amplitude = 0.4 #Ampiezza sinusoide [m]   (in Turtlesim ho messo 1.5)
        self.w =  (2.0 * math.pi) / (self.distance / self.velocity) #w è la velocità angolare    

        #FRAME LOCALE
        #y(t) = Asin(w*t), w*time_steps = moltiplica l'intero array (ciascun elemento) per w, e poi np.sin() applica a ciascun elemento il seno
        self.y_coord = self.amplitude * np.sin(self.w * self.time_steps)
        #x(t) = 0 (movimento solo lungo l'asse y locale)
        self.x_coord = self.time_steps * self.velocity * 0. # = np.zeros_like(y_coord)       

        #Velocità (derivate), y'(t) = w*Acos(w*t)
        self.y_coord_dot = self.w * self.amplitude * np.cos(self.w * self.time_steps)
        #x'(t) = 0 (non c'è movimento lungo l'asse x)
        self.x_coord_dot = np.ones_like(self.y_coord_dot) * self.velocity * 0. #x_coord_dot = np.zeros_like(y_coord_dot) 

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
    
#SPIEGAZIONE DETTAGLIATA:
#Abbiamo definito una traiettoria parametrica, nel frame locale, del tipo: x(t)=0, y(t)=Asin(wt) 
#(ovviamente relativo al punto anteriore).
#E di conseguenza x^.(t)=0, y^.(t)=w*Acos(wt), con A ampezza della sinsuoide e w la velocità angolare.
#Il che significa che il punto anteriore non avanza lungo l'asse x, ma oscilla su e giù.                                 
#Ovviamente nel frame globale l'oscillazione avverà lungo una retta di pendenza theta passante per 
#la posizione iniziale del robot.
#Abbiamo definito una velocità self.velocity=0.4 e una distanza self.distance=9.0. 
#E settiamo T=d/v (cioè mi calcolo il tempo che ci vuole a percorrere questa) e fissiamo quindi 
#w = 2*pi / T = 2pi / (d / v).
#Qui distance non è la lunghezza del percorso, stiamo solo dicendo voglio che un periodo della sinusoide
#duri T=d/v secondi, quindi è un modo indiretto per fissare T (un po' come sulla retta quando facevamo distanza/velocità)   
#La distanza percorsa realmente dal punto è:  
#quando una traiettoria è data come funzione nel tempo p(t)=(x(t),y(t)), la lunghezza del tratto percorso
#tra t0 e t1 è L = integrale da t0 a t1 di ||p^.(t)||dt = integrale da t0 a t1 di radice quadrata di x^.(t)^2 + y^.(t)^2 dt
#Il quale ci indica quanto cammini nello spazio, è la somma degli spostamenti infinitesimi (interale della velocità è la posizione!!).  
#nel nostro caso x^.(t) = 0, mentre y^.(t)=w*Acos(wt) ed il risultato è proprio 4A  ed è giusto così come abbiamo visto.
#Il modulo della velocità nel punto massimo vale A*w = 0.42, è un qualcosa a cui bisogna porre attenzione per non risichiare alte velocità
#e far faticare l'attuatore.
#Quindi il tempo che ci impiega è circa quello del periodo in quanto:
#il robot si muove seguendo le variazioni di velocità che gli passo dal controllo.
#Anche se non segue perfettamente ogni punto della traiettoria, avendo comunque una 
#velocità massima intorno a 0.4 m/s (come quella del riferimento), ci mette più o meno 
#lo stesso tempo a completare il percorso.
#Quindi il tempo totale è coerente con quello previsto dal riferimento, solo con piccole differenze 
#dovute al movimento reale.