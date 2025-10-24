#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, Point, PoseStamped
from turtlesim.msg import Pose
from robot_controller.utils.utility import wait_for_message 
from robot_controller.trajectories.straight import Straight
from robot_controller.trajectories.sinusoid import Sinusoid
from robot_controller.trajectories.circle import Circle
from robot_controller.trajectories.eight import Eight
from robot_controller.trajectories.square import Square
from robot_controller.trajectories.waypoint_linear import WaypointLinear
from robot_controller.trajectories.waypoint_smooth import WaypointSmooth
import numpy as np
from numpy import random
import math
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time



class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node_turtlesim') #Nome nodo

        #Parametri generali di controllo
        self.front_point = 0.1 #Punto anteriore (distanza) [m]
        #Guadagni proporzionali
        self.kx = 1.0
        self.ky = 1.0

        #Stato interno del robot [x, y, theta]
        self.current_pose = np.zeros((3,), dtype=float) #crea e restituisce un array numpy (class numpy.darray) contenti 3 valori nulli (0.)

        #Frequenza 
        self.hz = 10.0 #[Hz]
        
        #Liste contenenti posizione e velocità desiderate (riferimento) del punto anteriore,  
        #seguiranno una particolare traiettoria scelta dall'utente
        self.ref_position = []
        self.ref_velocities = []

        #Indice relativo allo scorrimento delle liste sopra nominate
        self.traj_index = 0 #inizializzato a zero in modo da esaminare subito il primo elemento

        #Matrice di rotazione
        self.rotation_matrix = np.zeros((2,2), dtype = float)
        #Anch'essa è vuota inizialmente ed è un array numpy (in particolare è una matrice).
        #La setteremo nel momento in cui leggeremo lo yaw iniziale

        #Publisher per il Controllo (v,w) + Debug (vediamo meglio poi)
        self.cmd_vel_pub = self.create_publisher(Twist, 
                                                 '/turtle1/cmd_vel', 
                                                 10)
        self.ref_signal_pub = self.create_publisher(PointStamped, 
                                                    '/ref_signal', 
                                                    10)
        self.xb_signal_pub = self.create_publisher(PointStamped, 
                                                   '/xb_signal', 
                                                   10)

        #Legame tra i due mondi: RViz e Turtlesim
        self.OFFSET_TURTLESIM = np.array([-5.5, -5.5]) 

        #Creiamo nuovi publisher i quali pubblicherrano non PointStamped, ma Path
        #(interi percorsi). Li ho inseriti per poter rappresentare in RViz2 l'intera traiettoria
        self.path_ref_signal_pub = self.create_publisher(Path,
                                                     'ref_path',
                                                     10)
        self.path_xb_signal_pub = self.create_publisher(Path,
                                                        'real_path',
                                                        10)
        #Creiamo due oggetti Path in cui andiamo a salvare le varie pose
        self.ref_path = Path()
        self.real_path = Path()
        
        #Input utente per selezionare traiettoria
        self.trajectory_type = input("Inserisci una traiettoria [retta, sinusoide, cerchio, otto, quadrato, spezzata, smooth]: ").strip().lower()
        #Il metodo strip() toglie eventuali spazi iniziali e finali, mentre
        #lower() rende tutti i caratteri minuscoli
        
        #Creazione della traiettoria desiderata (relativa al punto anteriore)
        self.compute_reference_trajectory(True)

        #Subscriber (Odometria/Pose) che peremtte di leggere la posa corrente del robot fisico (frame globale)
        self.create_subscription(Pose, #tipo msg
                                 '/turtle1/pose',  #topic
                                 self.pose_updater, #callback
                                 10) #QoS
        
        #Timer (la funzione di callback viene eseguita in questo caso ogni 0.1 secondi).
        #Sostituisce il rate.sleep() di ROS (è l'equivalente)
        #Questa callback è qulla che genera il controllo
        self.timer = self.create_timer(1.0 / self.hz, 
                                       self.control_loop) #callback
        
    
    def compute_reference_trajectory(self, first: bool):
        if first:
            #Traslazione iniziale
            init_offset = self.set_initial_variables() #restituisce un array numpy di shape (2,). 
                                                   #Usiamo un array numpy così facciamo calcoli in maniera veloce
        else:
            #Rinizializziamo le variabili iniziali (il subscriber continua ad andare)
            init_offset = self.reinitialization_params()

        #Verifico la tipologia di traiettoria scelta
        if self.trajectory_type == "retta":
            #Retta
            straight = Straight(hz=self.hz, 
                                rotation_matrix=self.rotation_matrix, 
                                init_offset=init_offset)
            self.ref_position, self.ref_velocities = straight.to_global_frame()

        elif self.trajectory_type == "sinusoide":
            #Sinusoide
            sinusoid = Sinusoid(hz=self.hz,
                                rotation_matrix=self.rotation_matrix,
                                init_offset=init_offset)
            self.ref_position, self.ref_velocities = sinusoid.to_global_frame()

        elif self.trajectory_type == "cerchio":
            #Cerchio
            circle = Circle(hz=self.hz,
                            rotation_matrix=self.rotation_matrix,
                            init_offset=init_offset)
            self.ref_position, self.ref_velocities = circle.to_global_frame()

        elif self.trajectory_type == "otto":
            eight = Eight(hz=self.hz,
                          rotation_matrix=self.rotation_matrix,
                          init_offset=init_offset)
            self.ref_position, self.ref_velocities = eight.to_global_frame()

        elif self.trajectory_type == 'quadrato':
            square = Square(hz=self.hz,
                            rotation_matrix=self.rotation_matrix,
                            init_offset=init_offset)
            self.ref_position, self.ref_velocities = square.to_global_frame()

        elif self.trajectory_type == "spezzata":
            waypoint_linear = WaypointLinear(
                                        hz=self.hz,
                                        rotation_matrix=self.rotation_matrix,
                                        init_offset=init_offset
                                        )
            self.ref_position, self.ref_velocities = waypoint_linear.to_global_frame()
        
        elif self.trajecotry_type == "smooth":
            waypoint_smooth = WaypointSmooth(
                                        hz=self.hz,
                                        rotation_matrix=self.rotation_matrix,
                                        init_offset=init_offset
                                        )
            self.ref_position, self.ref_velocities = waypoint_smooth.to_global_frame()

        else:
            self.get_logger().error("Traiettoria non supportata")
            self.destroy_node()
            rclpy.shutdown() #Ferma tutti i nodi attivi e libera le risorse (socket, connessioni DDS, memoria)
            return
        
        self.get_logger().info(f"Traiettoria '{self.trajectory_type}' generata correttamente")
    
    #Inizializzazione variabili (rotation_matrix, initial theta, ..)
    def set_initial_variables(self):
        #Topic name e timeout
        topic_name = '/turtle1/pose'
        timeout = 10.0 #[s]
        #Catturiamo il primo messaggio valido
        init_msg = wait_for_message(node=self, 
                                    msg_type=Pose, #Tipologia di messaggi, ha 5 campi tra cui di nostro interesse:
                                                   #float64 x, float64 y, float64 theta. 
                                                   #OSS: questo è Pose.msg relativo al pacchetto turtlesim.msg, il che è
                                                   #differente dal Pose.msg definito in geometry_msgs.msg
                                    topic_name=topic_name,
                                    hz=self.hz,
                                    timeout=timeout)

        #Gestione della casistica in cui venga restituito None
        if init_msg is None:
            self.get_logger().error(f"Nessun messaggio registrato dal topic {topic_name} in {timeout} secondi")
            self.destroy_node() #Distruggiamo il nodo
            rclpy.shutdown() #Termina l'esecuzione dello spin ed eseguirà il blocco finally
            return #interrompiamo l'esecuzione di questo metodo altrimenti proseguirebbe sotto

        #Inizializziamo le variabili relative allo stato attuale nel robot fisico (frame globale)
        init_theta = init_msg.theta #yaw (è rispetto l'asse x globale)
        #Posizione iniziale del punto anteriore
        init_x = init_msg.x + self.front_point * math.cos(init_theta) 
        init_y = init_msg.y + self.front_point * math.sin(init_theta)

        #Creiamo la matrice di rotazione che ci servirà per adattare frame locale a quello globale
        self.rotation_matrix = np.array([
            [math.cos(init_theta), -math.sin(init_theta)],
            [math.sin(init_theta),  math.cos(init_theta)]
        ], dtype=float)

        return np.array([init_x, init_y], dtype=float)

    #Callback subscriber, cattura la posizione corrente del robot 
    #e modifica l'array current_pose
    def pose_updater(self, msg: Pose):
        #Coordiante x e y del centro del robot
        self.current_pose[:]= [msg.x, msg.y, msg.theta] #sto modificando l'intero contenuto dell'array

    #Legge di controllo disaccoppiante
    def control(self, x_ref_fp: float, y_ref_fp: float, x_ref_fp_dot: float, y_ref_fp_dot: float): #Desiderate

        #Orientamento attuale
        theta = self.current_pose[2]
        #Posizione attuale del punto anteriore
        x_front_point = self.current_pose[0] + self.front_point * math.cos(theta)
        y_front_point = self.current_pose[1] + self.front_point * math.sin(theta)

        #Matrice di accoppiamento
        D = np.array([
            [math.cos(theta), -self.front_point * math.sin(theta)],
            [math.sin(theta),  self.front_point * math.cos(theta)]
            ], dtype = float)
        
        #Matrice inversa (legge di disaccoppiamento)
        D_inv = np.linalg.inv(D)

        #Controllo virtuale, u=[u1,u2] (shape (2,))
        control_matrix = np.array([
            -self.kx * (x_front_point - x_ref_fp) + x_ref_fp_dot,
            -self.ky * (y_front_point - y_ref_fp) + y_ref_fp_dot
        ], dtype = float)

        #self.get_logger().info(f"{self.traj_index} diff. pos. (x,y) = ({x_front_point - x_ref_fp}, {y_front_point - y_ref_fp})")

        #(v,w), 
        v_c, w_c = np.dot(D_inv, control_matrix).reshape((2,)) #Inutile questo rashape (per analoghi motivi), lo mettiamo solo per dare robustezza
        #self.get_logger().info(f"{self.traj_index} (v,w)=({v_c},{w_c})")

        return v_c, w_c

    #Arresto del robot, l'idea è di inviare più messaggi per far fermare il robot
    def stop_node(self):
        stop_msg = Twist() #messaggio di Twist vuoto 
        for i in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec= 1.0/ self.hz) #In realtà lo spin è già attivo, lo facciamo solo per titardare di 0.1 s la pubblicazione
        self.get_logger().info("Il robot è stato fermato correttamente")
    
    #Callback timer, ogni azione qua dentro viene eseguita ogni 0.1 s (frequenza)
    def control_loop(self):
        #Verifico se tutti gli elemebti della lista sono stati analizzati
        if self.traj_index >= len(self.ref_position): 
            self.get_logger().info('Traiettoria richiesta completata')
            self.stop_node()
            #Inetrrompiamo il timer mentre i pub e sub rimangono attivi
            self.timer.cancel()
            new_traj = input('Vuoi fare una nuova traiettoria ??\nIn caso scegli tra quelle precedenti, ' \
                            'altrimenti premi exit se non vuoi continuare: ').strip().lower()
            
            #Se ha digitato exit
            if new_traj == 'exit':
                self.get_logger().info('Stop simulazione')
                self.destroy_node()
                rclpy.shutdown()
                return
            
            #Traiettoria inesistente (errore battitura o altro)
            if new_traj not in['retta','cerchio', 'sinusoide', 'otto', 'quadrato', 'spezzata','smooth', 'exit']:
                self.get_logger().error("Traiettoria non supportata")
                self.destroy_node()
                rclpy.shutdown() #Ferma tutti i nodi attivi e libera le risorse (socket, connessioni DDS, memoria)
                return

            #Resettiamo i parametri
            self.reset_params(new_traj=new_traj)

            #Generiamo la nuova traiettoria desiderata
            self.compute_reference_trajectory(False) #non è più la prima volta
            
            #Riattiviamo il timer
            self.timer = self.create_timer(1.0 / self.hz, 
                                       self.control_loop)
            return #molto importante altrimento continua ad eseguire cose sotto che non vogliamo

        #Prendiamo i valori correnti da esaminare ()
        current_point_ref = self.ref_position[self.traj_index]
        current_point_dot_ref = self.ref_velocities[self.traj_index]

        v, w = self.control(current_point_ref[0], current_point_ref[1], 
                            current_point_dot_ref[0], current_point_dot_ref[1])

        #Creiamo il messaggio Twist per controllare velocità del robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(v) #per sicurezza alcuni middleware non amano tipi numpy.float64
        cmd_vel_msg.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd_vel_msg)

        #In ROS2 ogni nodo ha un suo oggetto clock che può usare il tempo del sistema
        #reale, oppure con il tempo simulato (qualche simulatore).
        #Tramite get_clock() otteniamo il clock associato al nodo e concatendnado now() 
        #otteniamo un oggetto di tipo Time il quala ha due parametri: tempo in secondi e in nanosecondi
        #Questa informazione la vogliamo mettere nell'header dei messaggi i quali però voglio
        #un messaggio di tipo Time (definito in un qualche modo) e non direttamente l'oggetto.
        #Il metodo to_msg() fa proprio questo
        current_time = self.get_clock().now().to_msg()

        #Pubblica riferimento (desiderata)
        self.publish_reference_position(current_time=current_time,
                                        x_ref=current_point_ref[0],
                                        y_ref=current_point_ref[1])

        #Pubblica posizione reale del punto anteriore
        self.publish_current_position(current_time=current_time)

        #Pubblichiamo i differenti path: quello di riferimento e quello effettuato
        self.publish_reference_path(current_time=current_time,
                                    x_ref=current_point_ref[0],
                                    y_ref=current_point_ref[1])
        self.publish_current_path(current_time=current_time)

        #Incremento
        self.traj_index += 1

    def reset_params(self, new_traj: str):
        self.trajectory_type = new_traj
        self.traj_index = 0
        self.ref_position.clear()
        self.ref_velocities.clear()
    
    def reinitialization_params(self):
        #yaw attuale
        init_theta = self.current_pose[2]
        #Calcoliamo la posizione iniziale del punto anteriore attuale
        init_x = self.current_pose[0] + self.front_point * math.cos(init_theta)
        init_y = self.current_pose[1] + self.front_point * math.sin(init_theta)

        #Modifichiamo la matrice di rotazione rispetto lo stato attuale
        self.rotation_matrix = np.array([
                [math.cos(init_theta), -math.sin(init_theta)],
                [math.sin(init_theta),  math.cos(init_theta)]
            ], dtype=float)
        
        return np.array([init_x, init_y], dtype=float)
    
    def publish_reference_position(self,current_time: Time, x_ref: float, y_ref: float):
        current_ref_msg = PointStamped()
        current_ref_msg.header.stamp = current_time
        current_ref_msg.header.frame_id = 'world' #Servirà per RViz2
        current_ref = Point()
        
        current_ref.x = x_ref + self.OFFSET_TURTLESIM[0]
        current_ref.y = y_ref + self.OFFSET_TURTLESIM[1]

        #Questa parte è stata sostituita da qulla sopra
        #current_ref.x = x_ref
        #current_ref.y = y_ref
        current_ref_msg.point = current_ref
        self.ref_signal_pub.publish(current_ref_msg)

    def publish_current_position(self, current_time: Time):
        current_xb_msg = PointStamped()
        current_xb_msg.header.stamp = current_time
        current_xb_msg.header.frame_id = 'world' #Servirà per RViz2
        current_xb = Point()
        
        current_xb.x = (self.current_pose[0] + self.front_point * math.cos(self.current_pose[2])) + self.OFFSET_TURTLESIM[0]
        current_xb.y = (self.current_pose[1] + self.front_point * math.sin(self.current_pose[2])) + self.OFFSET_TURTLESIM[1]

        #Questa parte è stata sostituita da qulla sopra
        #current_xb.x = self.current_pose[0] + self.front_point * math.cos(self.current_pose[2])
        #current_xb.y = self.current_pose[1] + self.front_point * math.sin(self.current_pose[2])
        current_xb_msg.point = current_xb
        self.xb_signal_pub.publish(current_xb_msg)
    
    def publish_reference_path(self, current_time: Time, x_ref: float, y_ref: float):
        pose_ref = PoseStamped()
        pose_ref.header.stamp = current_time
        pose_ref.header.frame_id = 'world'
        #Posizione
        pose_ref.pose.position.x = x_ref + self.OFFSET_TURTLESIM[0]
        pose_ref.pose.position.y = y_ref + self.OFFSET_TURTLESIM[1]
        #Orientamento (quaternion, occorre per forza scrivere qualcosa altrimento si avrebbe un valore non valido e si verifica un errore)
        #In questo modo stiamo dicendo nessuna rotazione.
        pose_ref.pose.orientation.w = 1.0

        self.ref_path.poses.append(pose_ref)
        self.ref_path.header.stamp = current_time
        self.ref_path.header.frame_id = 'world'

        self.path_ref_signal_pub.publish(self.ref_path)

    def publish_current_path(self, current_time: Time):
        pose_real = PoseStamped()
        pose_real.header.stamp = current_time
        pose_real.header.frame_id = "world"

        #Posizione
        pose_real.pose.position.x = (self.current_pose[0] + self.front_point * math.cos(self.current_pose[2])) + self.OFFSET_TURTLESIM[0]
        pose_real.pose.position.y = (self.current_pose[1] + self.front_point * math.sin(self.current_pose[2])) + self.OFFSET_TURTLESIM[1]
        #Orientamento (quaternion, occorre per forza scrivere qualcosa altrimento si avrebbe un valore non valido e si verifica un errore)
        #In questo modo stiamo dicendo nessuna rotazione.
        pose_real.pose.orientation.w = 1.0

        self.real_path.poses.append(pose_real)
        self.real_path.header.stamp = current_time
        self.real_path.header.frame_id = "world"
        
        self.path_xb_signal_pub.publish(self.real_path)

#Main
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrotto da tastiera")
    finally:
        node.stop_node() #se viene eseguito lo shutdown sopra non verrà eseguito
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()