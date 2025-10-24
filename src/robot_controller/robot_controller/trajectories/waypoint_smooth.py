import numpy as np
import math


class WaypointSmooth:
    def __init__(self, hz: float, rotation_matrix: np.ndarray, init_offset: np.ndarray, waypoints: list):

        self.waypoints = waypoints
        
        self.control_points = [] #Lista che conterrà tutti i punti di controllo

        self.velocity = 0.07 #Velocità desiderata lungo il percorso
        self.hz = hz #Frequenza di lavoro

        #Parametri per la conversione al frame globale
        self.rotation_matrix = rotation_matrix
        self.init_offset = init_offset

        
        #Parametri curva di bezier
        self.bezier_params = [] #Lista contente i valori "intelligenti" di lambda_smooth e curvature gain in ogni casistica
        self.lambda_smooth = 0.0 #[m] quanto taglio prima la retta per poi concatenarci la curva di Bezier (inizializzazione)
        self.curvature_gain = 0.0 #[m] posizione dei due punti di controllo per generare la curva (inizializzazione)
        #Indice per scrollare la lista sopra citata
        self.scroll_bezier_params_list = 0

        #Liste contenente le coordinate x(t) e y(t) di ogni retta "tagliata" (cioè dove ogni retta deve essere)
        self.x_coord_cut_straight = []
        self.y_coord_cut_straight = []

        #Liste contenti le velocità da assumere ad ogni tratto di retta (è solo per comodità in qaunto per oggni retta andiamo alla stessa velocità)
        self.x_velocity_cut_straight = []
        self.y_velocity_cut_straight = []

        #Liste contenenti le coordinate x(t) e y(t) delle curve di Bezier
        self.x_coord_bezier_curve = []
        self.y_coord_bezier_curve = []

        #Liste contenenti le velocità da assumere nelle curve di Bezier
        self.x_velocity_bezier_curve = []
        self.y_velocity_bezier_curve = []

        #Array contenenti posizioni e velocità finali dell'intera traiettoria (in locale)
        self.x_coord = np.array([], dtype=float)
        self.y_coord = np.array([], dtype=float)
        self.x_coord_velocity = np.array([], dtype=float)
        self.y_coord_velocity = np.array([], dtype=float)

        #Posizione e velocità (global frame, da passare al nodo controllore)
        self.ref_position = []
        self.ref_velocities = []

        #self.test =[]

        self.compute_trajectory()
    
    def compute_trajectory(self):
        self.calculate_angle()
        self.compute_straight_and_save_control_points()
        self.create_bezier_curves()
        self.complete_trajectory_compute()
    
    def compute_straight_and_save_control_points(self):
        #Il numero di rette è pari al numero di waypoint - 1
        for i in range(len(self.waypoints) - 1):
            first_index_waypoint = i
            second_index_waypoint = i + 1

            if first_index_waypoint == 0:

                #Modifichiamo i parametri della curva di Bezier
                self.lambda_smooth = self.bezier_params[self.scroll_bezier_params_list][0]
                self.curvature_gain = self.bezier_params[self.scroll_bezier_params_list][1]

                #self.test.append([self.lambda_smooth, self.curvature_gain])

                #Se è la prima retta occorre interrompere solo il pezzo finale e quindi bisogna "spostare" solo l'ultimo waypoint
                new_second_waypoint, versor = self.change_second_waypoint(self.waypoints[first_index_waypoint], self.waypoints[second_index_waypoint]) #Ho aggiunto successivamente il versore per calcolarmi il secondo punto di controllo relativo al primo tratto
                
                #Distanza tra due waypoint
                distance_waypoints = math.sqrt((new_second_waypoint[0] - self.waypoints[first_index_waypoint][0]) ** 2 
                                        + (new_second_waypoint[1] - self.waypoints[first_index_waypoint][1]) ** 2)

                #Tempo di percorrenza della distanza
                period = distance_waypoints / self.velocity

                #Array tempi discreti 
                time_steps = np.arange(start=0.0, step=1.0 / self.hz , stop=period + (1.0 / self.hz)) #Così includo anche l'ultimo valore se è preciso,
                                                                                              #altrimenti possono esserci delle imprecisioni:
                                                                                              #l'ultimo valore può non essere esatto, non coincidere
                                                                                              #potremmo forzarlo.
                time_steps[-1] = period

                #Distanza da percorre ad ogni istante di tempo lungo la traiettoria
                space_steps = time_steps * self.velocity

                x_coord_segment = self.waypoints[first_index_waypoint][0] + (space_steps / distance_waypoints) * (new_second_waypoint[0] - self.waypoints[first_index_waypoint][0])
                y_coord_segment = self.waypoints[first_index_waypoint][1] + (space_steps / distance_waypoints) * (new_second_waypoint[1] - self.waypoints[first_index_waypoint][1])

                #Aggiungiamo queste informazioni alla lista (si avrà anche un ordine logico derivativo)
                self.x_coord_cut_straight.append(x_coord_segment)
                self.y_coord_cut_straight.append(y_coord_segment)

                #Mi calcolo la velocità che bisogna dare per percorrere quel tratto di retta
                #Vedi ragionamento (sono valori scalari in quanto costanti)
                x_velocity_segment = self.velocity * ((new_second_waypoint[0] - self.waypoints[first_index_waypoint][0]) / distance_waypoints) #vx lungo la traiettoria
                y_velocity_segment = self.velocity * ((new_second_waypoint[1] - self.waypoints[first_index_waypoint][1]) / distance_waypoints) #vy lungo la traiettoria

                #Creo degli array che mi indicano la velocità che devo assumere in ogni istante di tempo (è costante quindi si avranno gli stessi valori)
                x_velocity_segment_array = np.full(len(x_coord_segment), x_velocity_segment)
                y_velocity_segment_array = np.full(len(y_coord_segment), y_velocity_segment)

                #Aggiungiamo queste informazioni alle lista
                self.x_velocity_cut_straight.append(x_velocity_segment_array)
                self.y_velocity_cut_straight.append(y_velocity_segment_array)

                #Quello che bisogna fare ora è fissare i punti di controllo per generare poi le curve di Bezier
                #Il primo punto di controllo per unire il primo e secondo segmento si trova alla fine di questa prima retta
                first_control_point = [x_coord_segment[-1], y_coord_segment[-1]]
                #Il successivo punto di controllo è lungo la traiettoria della prima retta, e si trova a distanza self.curvature_gain da questa.
                #Abbiamo a disposizione il versore: puntoFinaleRetta + self.curvature_gain * u
                second_control_point = [new_second_waypoint[0] + versor[0] * self.curvature_gain,
                                        new_second_waypoint[1] + versor[1] * self.curvature_gain]
                #Aggiungiamo questi punti di controllo alla lista, li mettiamo in ordine almeno è già tutto per bene 
                self.control_points.append(first_control_point)
                self.control_points.append(second_control_point)

            #Se è l'ultima retta occorre tagliare solamente il tratto iniziale
            elif first_index_waypoint == (len(self.waypoints) - 2):

                #self.test.append([self.lambda_smooth, self.curvature_gain])
                
                #dell'ultima retta abbiamo già impostato i valori corretti dei parametri
                new_first_waypoint, versor = self.change_first_waypoint(self.waypoints[first_index_waypoint], self.waypoints[second_index_waypoint])
                #Distanza tra due waypoint
                distance_waypoints = math.sqrt((self.waypoints[second_index_waypoint][0] - new_first_waypoint[0]) ** 2 
                                        + (self.waypoints[second_index_waypoint][1] - new_first_waypoint[1]) ** 2)

                #Tempo di percorrenza della distanza
                period = distance_waypoints / self.velocity

                #Array tempi discreti 
                time_steps = np.arange(start=0.0, step=1.0 / self.hz , stop=period + (1.0 / self.hz)) #Così includo anche l'ultimo valore se è preciso,
                                                                                              #altrimenti possono esserci delle imprecisioni:
                                                                                              #l'ultimo valore può non essere esatto, non coincidere
                                                                                              #potremmo forzarlo.
                time_steps[-1] = period

                #Distanza da percorre ad ogni istante di tempo lungo la traiettoria
                space_steps = time_steps * self.velocity

                x_coord_segment = new_first_waypoint[0] + (space_steps / distance_waypoints) * (self.waypoints[second_index_waypoint][0] - new_first_waypoint[0])
                y_coord_segment = new_first_waypoint[1] + (space_steps / distance_waypoints) * (self.waypoints[second_index_waypoint][1] - new_first_waypoint[1])

                #Aggiungiamo queste informazioni alla lista (si avrà anche un ordine logico derivativo)
                self.x_coord_cut_straight.append(x_coord_segment)
                self.y_coord_cut_straight.append(y_coord_segment)

                #Mi calcolo la velocità che bisogna dare per percorrere quel tratto di retta
                #Vedi ragionamento (sono valori scalari in quanto costanti)
                x_velocity_segment = self.velocity * ((self.waypoints[second_index_waypoint][0] - new_first_waypoint[0]) / distance_waypoints) #vx lungo la traiettoria
                y_velocity_segment = self.velocity * ((self.waypoints[second_index_waypoint][1] - new_first_waypoint[1]) / distance_waypoints) #vy lungo la traiettoria

                #Creo degli array che mi indicano la velocità che devo assumere in ogni istante di tempo (è costante quindi si avranno gli stessi valori)
                x_velocity_segment_array = np.full(len(x_coord_segment), x_velocity_segment)
                y_velocity_segment_array = np.full(len(y_coord_segment), y_velocity_segment)

                #Aggiungiamo queste informazioni alle lista
                self.x_velocity_cut_straight.append(x_velocity_segment_array)
                self.y_velocity_cut_straight.append(y_velocity_segment_array)

                #Nel caso in cui siamo nell'ultima retta abbiamo sempre di punti di controllo da tenere a mente, quelli all'inizio.
                first_control_point = [new_first_waypoint[0] - versor[0] * self.curvature_gain,
                                       new_first_waypoint[1] - versor[1] * self.curvature_gain]
                second_control_point = [x_coord_segment[0], y_coord_segment[0]]

                #Aggiungiamo questi punti di controllo alla lista, li mettiamo in ordine almeno è già tutto per bene 
                self.control_points.append(first_control_point)
                self.control_points.append(second_control_point)



            else: #Per quanto riguada gli altri tratti, li devo far partire self.lambda_smooth dopo in quanto li taglio anche all'inizio
                #Quello che devo fare è spostare il punto iniziale e finale di ogni retta, in modo da tagliarla. Per farlo definisco nuovi waypoint che sono spostati di d lungo la direzione
                #Dato che la parte iniziale di retta è legata alla parte finale della precedente i parametri della curva di bezier devono essere i medesimi
                
                #self.test.append([self.lambda_smooth, self.curvature_gain])
                
                new_first_waypoint, versor = self.change_first_waypoint(self.waypoints[first_index_waypoint], self.waypoints[second_index_waypoint])
                #Troviamo subito il primo punto di controllo
                first_control_point = [new_first_waypoint[0] - versor[0] * self.curvature_gain,
                                       new_first_waypoint[1] - versor[1] * self.curvature_gain]
                
                #Modifichiamo i parametri della curva di Bezier
                self.scroll_bezier_params_list += 1
                self.lambda_smooth = self.bezier_params[self.scroll_bezier_params_list][0]
                self.curvature_gain = self.bezier_params[self.scroll_bezier_params_list][1]

                #self.test.append([self.lambda_smooth, self.curvature_gain])

                #Calcoliamoci il secondo nuovo waypoint secondi i nuovi parametri
                new_second_waypoint, _ = self.change_second_waypoint(self.waypoints[first_index_waypoint], self.waypoints[second_index_waypoint])
                
                #Distanza tra due i due nouvi waypoint
                distance_waypoints = math.sqrt((new_second_waypoint[0] - new_first_waypoint[0]) ** 2 
                                        + (new_second_waypoint[1] - new_first_waypoint[1]) ** 2)
                
                #Tempo di percorrenza della distanza
                period = distance_waypoints / self.velocity
                #Array tempi discreti 
                time_steps = np.arange(start=0.0, step=1.0 / self.hz , stop=period + (1.0 / self.hz)) #Così includo anche l'ultimo valore se è preciso,
                                                                                              #altrimenti possono esserci delle imprecisioni:
                                                                                              #l'ultimo valore può non essere esatto, non coincidere
                                                                                              #potremmo forzarlo.
                time_steps[-1] = period

                #Distanza da percorre ad ogni istante di tempo lungo la traiettoria
                space_steps = time_steps * self.velocity

                #Coordinate relative alla posizione di un segmento. Nel primo tratto non devo fare nulla, si interrompe prima in modo automatico in qaunto gli ho tagliato un pezzo di distanza
                x_coord_segment = new_first_waypoint[0] + (space_steps / distance_waypoints) * (new_second_waypoint[0] - new_first_waypoint[0])
                y_coord_segment = new_first_waypoint[1] + (space_steps / distance_waypoints) * (new_second_waypoint[1] - new_first_waypoint[1])

                #Aggiungiamo queste informazioni alla lista (si avrà anche un ordine logico derivativo)
                self.x_coord_cut_straight.append(x_coord_segment)
                self.y_coord_cut_straight.append(y_coord_segment)

                #Calcoliamoci la velocità che deve assumere il robot lungo il tratto
                x_velocity_segment = self.velocity * ((new_second_waypoint[0] - new_first_waypoint[0]) / distance_waypoints) #vx lungo la traiettoria
                y_velocity_segment = self.velocity * ((new_second_waypoint[1] - new_first_waypoint[1]) / distance_waypoints) #vy lungo la traiettoria

                #Creo degli array che mi indicano la velocità che devo assumere in ogni istante di tempo (è costante quindi si avranno gli stessi valori)
                x_velocity_segment_array = np.full(len(x_coord_segment), x_velocity_segment)
                y_velocity_segment_array = np.full(len(y_coord_segment), y_velocity_segment)

                #Aggiungiamo queste informazioni alle lista
                self.x_velocity_cut_straight.append(x_velocity_segment_array)
                self.y_velocity_cut_straight.append(y_velocity_segment_array)

                #Calcoliamo i restanti punti di controllo (OSS:In questa casistica bisogna calcolare 4 punti di controllo: due all'inzio e due alla fine)
                second_control_point = [x_coord_segment[0], y_coord_segment[0]]
                third_control_point = [x_coord_segment[-1], y_coord_segment[-1]]
                fourth_control_point = [new_second_waypoint[0] + versor[0] * self.curvature_gain,
                                        new_second_waypoint[1] + versor[1] * self.curvature_gain]
                #Mettiamo in ordien all'interno della lista
                self.control_points.append(first_control_point)
                self.control_points.append(second_control_point)
                self.control_points.append(third_control_point)
                self.control_points.append(fourth_control_point)

    
    def change_first_waypoint(self, first_waypoint: list, second_waypoint: list):
        distance_vector = [second_waypoint[0] - first_waypoint[0], #x2-x1
                           second_waypoint[1] - first_waypoint[1]] #y2-y1
        distance_waypoint = math.sqrt((second_waypoint[0] - first_waypoint[0]) ** 2 + (second_waypoint[1] - first_waypoint[1]) ** 2)
        # u = v / |v|, versore che indica la direzione
        versor = [distance_vector[0] / distance_waypoint, 
                  distance_vector[1] / distance_waypoint]
        #Il nuovo waypoint che si trova a distanza d da P1 lungo la direzione P1->P2 è Pd = P1 + d*u (vettore)
        change_first_waypoint = [first_waypoint[0] + self.lambda_smooth * versor[0],
                                  first_waypoint[1] + self.lambda_smooth * versor[1]]
        return change_first_waypoint, versor

    def change_second_waypoint(self, first_waypoint: list, second_waypoint: list):
        distance_vector = [second_waypoint[0] - first_waypoint[0], #x2-x1
                           second_waypoint[1] - first_waypoint[1]] #y2-y1
        distance_waypoint = math.sqrt((second_waypoint[0] - first_waypoint[0]) ** 2 + (second_waypoint[1] - first_waypoint[1]) ** 2)
        # u = v / |v|, versore che indica la direzione
        versor = [distance_vector[0] / distance_waypoint, 
                  distance_vector[1] / distance_waypoint]
        #Il nuovo waypoint che so trova a distanza -d da P2 lungo la direzione P1->P2 è Pd = P2 - d*u (vettore)
        change_second_waypoint = [second_waypoint[0] - self.lambda_smooth * versor[0],
                                   second_waypoint[1] - self.lambda_smooth * versor[1]]
        return change_second_waypoint, versor
    
    #L'obiettivo è creare curve di Bezier cubiche, sfruttando i punti di controllo individuati,
    #e individuarne posizione e velocità.
    def create_bezier_curves(self):
        for i in range(0, len(self.control_points) - 3, 4): #Considero 4 punti di controllo alla volta
            #Il primo passo è quella di calcolare la lunghezza della curva. Per farlo dovremmo fare un integrale
            #che varia da 0 a 1 di |dB(t)/dt| dt, cioè la somma di tutti i pezzettini della curva.
            #Però questo integrale non ha una forma chiusa e quindi quello che si fa è darne una stima.
            length = self.bezier_curve_length(P0=self.control_points[i],
                                     P1=self.control_points[i+1],
                                     P2=self.control_points[i+2],
                                     P3=self.control_points[i+3])
            #Ho ottenuto la lunghezza della curva, è come se fosse una retta.. e voglio trovare il tempo necessario a percorrerla con quella velocità
            period = length / self.velocity
            #Adesso biosgna fare un ragionamento differente dato che è definita tra [0,1].
            #Ho trovato il periodo, e conosco la frequenza.. quindi so il numero di campioni che devo prendere
            samples = max(20, int(period * self.hz))  #Evitiamo che i campioni possono essere 0 in linspace, inoltre
                                                       #devo trasformare il prodotto in int dato che può resituire un float.
                                                       #Per maggior efficenza puoi valutare se aumentare il numero 20
            #Quindi genero ora questi numeri di campioni equamente distribuiti tra [0,1]
            time_steps = np.linspace(start=0.0, stop=1.0, num=samples)
            #Andiamo a calcolare le coordinate x(t),y(t) (posizione)
            self.calculate_coords(time_steps=time_steps,
                                  P0=self.control_points[i],
                                  P1=self.control_points[i+1],
                                  P2=self.control_points[i+2],
                                  P3=self.control_points[i+3])
            self.calculate_velocity(time_steps=time_steps,
                                    P0=self.control_points[i],
                                    P1=self.control_points[i+1],
                                    P2=self.control_points[i+2],
                                    P3=self.control_points[i+3])

    #L'idea è quella di campinare la curva e misurare la distanza tra i vari campioni 
    #OSS: non è un campionamento uniforme, cioè non si mantiene la stessa distanza, nel tempo sono equispaziati ma nello spazio no (potremmo farlo per robustezza). 
    #Ricordo che la curva di Bezier è definita tra [0,1].
    def bezier_curve_length(self, P0: list, P1: list, P2: list, P3: list):
        samples = 500
        t = np.linspace(start=0.0, stop=1.0, num=samples)
        #La formula di una curva di Bezier cubica è: B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
        x = (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0]
        y = (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]
        #Calcoliamo la distanza
        length = 0
        for i in range(len(x) - 1):
            distance = math.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
            length += distance
        return length
    
    def calculate_coords(self, time_steps: np.ndarray, P0: list, P1: list, P2: list, P3: list):
        x_coord = (1 - time_steps)**3 * P0[0] + 3 * (1 - time_steps)**2 * time_steps * P1[0] + 3 * (1 - time_steps) * time_steps**2 * P2[0] + time_steps**3 * P3[0]
        y_coord = (1 - time_steps)**3 * P0[1] + 3 * (1 - time_steps)**2 * time_steps * P1[1] + 3 * (1 - time_steps) * time_steps**2 * P2[1] + time_steps**3 * P3[1]
        #Aggiungiamo alla lista
        self.x_coord_bezier_curve.append(x_coord)
        self.y_coord_bezier_curve.append(y_coord)
    
    #Occorre calcolare la velocità, ovverro la derivata di B(t), cioè B'(t).
    #La derivata di una curva di Bezier cubica è:
    #B'(t) = 3(1-t)^2 (P1 - P0) + 6 (1-t)t(P2 - P1) + 3t^2(P3 - P2)
    def calculate_velocity(self,time_steps: np.ndarray, P0: list, P1: list, P2: list, P3: list):
        x_velocity = 3 * (1 - time_steps)**2 * (P1[0] - P0[0]) + 6 * (1 - time_steps) * time_steps * (P2[0] - P1[0]) + 3 * time_steps**2 * (P3[0] - P2[0])
        y_velocity = 3 * (1 - time_steps)**2 * (P1[1] - P0[1]) + 6 * (1 - time_steps) * time_steps * (P2[1] - P1[1]) + 3 * time_steps**2 * (P3[1] - P2[1])
        #Quello che bisogna fare ora è normalizzare la velocità dato che vogliamo che questa sia pari a self.velocity
        #Mi calcolo il modulo del vettore velocità
        speed = np.sqrt(x_velocity**2 + y_velocity**2)
        eps = 1e-6 #epsilon, lo metto per evitare divisione per zero
        speed = np.maximum(speed, eps)
        #Faccio qui una proporzione, vogliamo che la direzione rimanga quella ma che il modulo desiderato sia self.velocity e vogliamo che sia costante.
        #Con questa formula è come dire: "prendi il vettore velocità e accorgialo fino a quando il suo modulo è pari a self.velocity"
        x_velocity = (x_velocity / speed) * self.velocity
        y_velocity = (y_velocity / speed) * self.velocity
        #Aggiungiamo alla lista
        self.x_velocity_bezier_curve.append(x_velocity)
        self.y_velocity_bezier_curve.append(y_velocity)
    
    def complete_trajectory_compute(self):
        self.delete_duplicate()
        self.concatenate_positions()
        self.concatenate_velocities()

    #Quello che bisogna fare ora è eliminare i punti duplicati. Cioè, per esempio, il primo punto della
    #prima curva di Bezier coincide con l'ultimo punto della prima rette e così via.. 
    def delete_duplicate(self):
        for i in range(len(self.x_coord_cut_straight)):
            #Se è la prima retta devo solo togliere l'ultimo elemento
            if i == 0:
                self.x_coord_cut_straight[i] = np.delete(self.x_coord_cut_straight[i], -1)
                self.y_coord_cut_straight[i] = np.delete(self.y_coord_cut_straight[i], -1)
                #Dovrò anche eliminare le velocità associate
                self.x_velocity_cut_straight[i] = np.delete(self.x_velocity_cut_straight[i], -1)
                self.y_velocity_cut_straight[i] = np.delete(self.y_velocity_cut_straight[i], -1)
            #Ultima retta, tolgo solo il primo
            elif i == (len(self.x_coord_cut_straight) - 1):
                self.x_coord_cut_straight[i] = np.delete(self.x_coord_cut_straight[i], 0)
                self.y_coord_cut_straight[i] = np.delete(self.y_coord_cut_straight[i], 0)
                #Dovrò anche eliminare le velocità associate
                self.x_velocity_cut_straight[i] = np.delete(self.x_velocity_cut_straight[i], 0)
                self.y_velocity_cut_straight[i] = np.delete(self.y_velocity_cut_straight[i], 0)
            #Altrimenti se si tratta di una retta in mezzo occore togliere entrambi
            else: 
                self.x_coord_cut_straight[i] = np.delete(self.x_coord_cut_straight[i], [0, -1])
                self.y_coord_cut_straight[i] = np.delete(self.y_coord_cut_straight[i], [0, -1])
                #Dovrò anche eliminare le velocità associate
                self.x_velocity_cut_straight[i] = np.delete(self.x_velocity_cut_straight[i], [0, -1])
                self.y_velocity_cut_straight[i] = np.delete(self.y_velocity_cut_straight[i], [0, -1])
    
    #Il numero di curve di bezier è pari al numero di rette -1 (bisogna ricordarsi ciò)
    def concatenate_positions(self):
        for i in range(len(self.x_coord_cut_straight)):
            #All'ultimo devo solo unire l'ultime retta e non anche la curva di bezier
            if i == (len(self.x_coord_cut_straight) - 1):
                self.x_coord = np.concatenate((self.x_coord, self.x_coord_cut_straight[i]))
                self.y_coord = np.concatenate((self.y_coord, self.y_coord_cut_straight[i]))
            else:
                self.x_coord = np.concatenate((self.x_coord, self.x_coord_cut_straight[i], self.x_coord_bezier_curve[i]))
                self.y_coord = np.concatenate((self.y_coord, self.y_coord_cut_straight[i], self.y_coord_bezier_curve[i]))


    def concatenate_velocities(self):
        for i in range(len(self.x_coord_cut_straight)):
            #All'ultimo devo solo unire l'ultime retta e non anche la curva di bezier
            if i == (len(self.x_coord_cut_straight) - 1):
                self.x_coord_velocity = np.concatenate((self.x_coord_velocity, self.x_velocity_cut_straight[i]))
                self.y_coord_velocity = np.concatenate((self.y_coord_velocity, self.y_velocity_cut_straight[i]))
            else:
                self.x_coord_velocity = np.concatenate((self.x_coord_velocity, self.x_velocity_cut_straight[i], self.x_velocity_bezier_curve[i]))
                self.y_coord_velocity = np.concatenate((self.y_coord_velocity, self.y_velocity_cut_straight[i], self.y_velocity_bezier_curve[i]))
    
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
    
    #Calcolo gli angoli interni, "scelta intelligente"
    def calculate_angle(self):
        for i in range(1, (len(self.waypoints) - 1)):
            #Calcoliamo l'angolo tra 3 waypoint, quindi per prima cosa individuamo i 3 waypoint da analizzare
            current_waypoint = np.array(self.waypoints[i], dtype=float)
            previous_waypoint = np.array(self.waypoints[i - 1], dtype=float)
            next_waypoint = np.array(self.waypoints[i + 1], dtype=float)
            #Calcoliamo i vettori u e v che partano dal punto centrale e vanno verso gli esterni
            u = previous_waypoint - current_waypoint
            v = next_waypoint - current_waypoint
            #Utilizziamo il prodotto scalare per calcolarci l'angolo interno
            scalar_product = np.dot(u, v) #ux * vx + uy * vy (il risultato è uno scalare)
            norm_u = math.sqrt(u[0]**2 + u[1]**2)
            norm_v = math.sqrt(v[0]**2 + v[1]**2)
            theta = np.degrees(np.arccos(scalar_product / (norm_u * norm_v)))
            if theta >= 90:
                self.bezier_params.append([
                    0.12, #lambda_smooth
                    0.06 #curvature_gain
                ])
            elif theta < 90 and theta >=50:
                self.bezier_params.append([
                    0.12, #lambda_smooth
                    0.04 #curvature_gain
                ])
            else:
                self.bezier_params.append([
                    0.15, #lamnda_smooth
                    0.03 #curvature_gain
                ])
    
    #Debug per vedere i parametri usati
    #def stamp(self):
    #    return self.test

#OSS: potrei amentare la robustezza mettendo un controllo sulla posizione dei waypoint in modo tare da
#non averli sovrapposti e avere quindi una distanza nulla che può causare errore (dato che la usiamo per dividere..)
#Che poi anche se non sono sovrapposti bisogna stare attenti a metterli ad una certa distanza perchè abbiamo i parametri della curva di Bezier