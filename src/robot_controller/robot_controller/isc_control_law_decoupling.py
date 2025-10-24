#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Trajectory
from geometry_msgs.msg import Twist, PointStamped, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from robot_controller.utils.utility import yaw_from_quaternion
from builtin_interfaces.msg import Time
import math
import numpy as np


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node_turtlebot')

        #Parametri generali di controllo
        self.front_point = 0.10 #Punto anteriore (distanza) [m]
        #Guadagni proporzionali
        self.kx = 1.0
        self.ky = 1.0

        #Stato interno del robot [x, y, theta]
        self.current_pose = np.zeros((3,), dtype=float) #crea e restituisce un array numpy (class numpy.darray) contenti 3 valori nulli (0.)

        #Frequenza (influenzerà il timer)
        self.hz = 20.0

        #Indice relativo allo scorrimento delle liste sopra nominate
        self.traj_index = 0 #inizializzato a zero in modo da esaminare subito il primo elemento

        #Liste contenenti posizione e velocità desiderate (riferimento) del punto anteriore,  
        #seguiranno una particolare traiettoria scelta dall'utente
        self.ref_position = []
        self.ref_velocities = []

        #Liste utili per il calcolo della metrica RMSE (ed in modo indiretto della MSE).
        #Salviamo solo i valori assunti dal robot reale (e non quelli di riferimento in quanto li abbiamo già a disposizione)
        self.rmse_x_real = []
        self.rmse_y_real = []

        #Publisher (invio di comandi (v,w))
        self.cmd_vel_pub = self.create_publisher(Twist, 
                                                 '/cmd_vel', 
                                                 10)
        #Publisher (debug traiettoria)
        self.ref_signal_pub = self.create_publisher(PointStamped, 
                                                    '/ref_signal', 
                                                    10)
        self.xb_signal_pub = self.create_publisher(PointStamped, 
                                                   '/xb_signal', 
                                                   10)
        
        #Creiamo anche nuovi publisher i quali pubblicherrano non PointStamped, ma Path (interi percorsi). 
        #Ho deciso di inserirli per poter rappresentare in RViz2 l'intera traiettoria
        self.path_ref_signal_pub = self.create_publisher(Path,
                                                     'ref_path',
                                                     10)
        self.path_xb_signal_pub = self.create_publisher(Path,
                                                        'real_path',
                                                        10)
        #Creiamo due oggetti Path in cui andiamo a salvare le varie pose
        self.ref_path = Path()
        self.real_path = Path()
        
        #Subscriber (Odometria)
        self.create_subscription(Odometry, #tipo msg
                                 '/odom',  #topic
                                 self.pose_updater, #callback
                                 10) #QoS queue
        
        #Lettura traiettoria
        self.create_subscription(Trajectory,
                                 '/reference_trajectory',
                                 self.trajectory_callback,
                                 10)
        
        #Timer (la funzione di callback viene eseguita in questo caso ogni 0.1 secondi).
        #Sostituisce il rate.sleep() di ROS (è l'equivalente)
        #Questa callback è qulla che genera il controllo
        self.timer = self.create_timer(1.0 / self.hz, 
                                       self.control_loop) #callback
        
        #Mi serve come contatore per capire ogni quanto inviare il messaggio di attesa
        self.wait_count = 0.0
        

    #Callback subscriber, cattura la posizione corrente del robot 
    # e modifica current_pose
    def pose_updater(self, msg: Odometry):
        #Coordiante x e y del centro del robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #Orientamento del robot (quaternione)
        quaternion = msg.pose.pose.orientation
        theta = yaw_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.current_pose[:]= [x, y, theta] #sto modificando l'intero contenuto dell'array
        #self.get_logger().info("Posa aggiornata")
    
    def trajectory_callback(self, msg: Trajectory):
        #In caso in cui viene inserita una nuova traiettoria anche se la precedente non è terminata
        self.reset_params()
        #Stop immediato per evitare che il robot “scivoli” con l’ultimo cmd_vel (sempre se abbiamo cambiato traiettoria nel mentre)
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        rclpy.spin_once(self, timeout_sec=1.0/self.hz) #Mi funge da pausa ed inoltre vengono aggiornate eventuali posizioni 
                                                       #(il timer non funziona in qaunto le liste sono vuote). Inoltre mi peremtte quindi di pubblicare effettivamente
                                                       #il messaggio che indica di fermarsi e si aspetta un pochino

        #Analizzo ogni lista
        for pos, vel in zip(msg.positions, msg.velocities):
            self.ref_position.append([pos.x, pos.y])
            self.ref_velocities.append([vel.x, vel.y])
        self.get_logger().info(f"Traiettoria ricevuta con successo")


    #Legge di controllo disaccoppiante
    def control(self, x_ref_b: float, y_ref_b: float, x_ref_b_dot: float, y_ref_b_dot: float): #Desiderate
        #Orientamento attuale
        theta = self.current_pose[2]
        #Posizione attuale del punto anteriore di riferimento
        xb = self.current_pose[0] + self.front_point * math.cos(theta)
        yb = self.current_pose[1] + self.front_point * math.sin(theta)

        #Matrice di accoppiamento
        D = np.array([
            [math.cos(theta), -self.front_point * math.sin(theta)],
            [math.sin(theta),  self.front_point * math.cos(theta)]
            ], dtype = float)
        
        #Matrice inversa (legge di disaccoppiamento)
        D_inv = np.linalg.inv(D)

        #Controllo virtuale, u=[u1,u2] (shape (2,))
        control_matrix = np.array([
            -self.kx * (xb - x_ref_b) + x_ref_b_dot,
            -self.ky * (yb - y_ref_b) + y_ref_b_dot
        ], dtype = float)

        #(v,w), 
        v_c, w_c = np.dot(D_inv, control_matrix)
        #OSS. Python tratta una shape (2,) come un vettore colonna, il risultato
        #di questa moltiplicazione (2,2)*(2,) è (2,)

        return v_c, w_c

    #Arresto del robot
    def stop_node(self):
        stop_msg = Twist() #messaggio di Twist vuoto (quindi ferma il robot)
        for i in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec= 1.0/ self.hz)
        self.get_logger().info("Il robot è stato fermato correttamente")


    #Callback timer, ogni azione qua dentro viene eseguita ogni 0.1 s (frequenza)
    def control_loop(self):
        #Verifico che sia stato ricevuto il msg dal nodo create_trajectory
        if len(self.ref_position) != 0 and len(self.ref_velocities) != 0:
            #Verifico se tutti gli elementi della lista sono stati analizzati
            if self.traj_index >= len(self.ref_position): 
                self.get_logger().info('Traiettoria richiesta completata')
                #Calcoliamo la metrica RMSE ed indirettamente la MSE
                self.calculate_rmse()
                #Fermiamo il robot
                self.stop_node()
                #Rinizializziamo i parametri in caso venga scelta una nuova traiettoria
                self.reset_params()
                return

            #Prendiamo i valori correnti da esaminare
            current_point_ref = self.ref_position[self.traj_index]
            current_point_dot_ref = self.ref_velocities[self.traj_index]

            #Calcoliamoci RMSE e MSE. Per non appesantire la mole di lavoro del controllore salviamo
            #solamente questi valori e alla fine della traiettoria ci calcoliamo le metriche.
            #Inoltre usiamo liste per salvare valori e non array perchè quest'ultimi sono più pesanti (quando devo aggiungere un elemento mi si crea un nuvo array..)
            #Orientazione attuale e posizione attuale del punto anteriore
            theta = self.current_pose[2]
            xb = self.current_pose[0] + self.front_point * math.cos(theta)
            yb = self.current_pose[1] + self.front_point * math.sin(theta)
            #Aggiungiamo alla lista
            self.rmse_x_real.append(xb)
            self.rmse_y_real.append(yb)

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

            #Incremento per poter poi analizzare il prossimo punto
            self.traj_index += 1
        else:
            timeout = 10.0 #[s]
            self.wait_count += 1
            if self.wait_count == (timeout * self.hz):
                self.get_logger().info("In attessa di ricevere una traiettoria")
                self.wait_count = 0.0


    def reset_params(self):
        self.traj_index = 0
        self.ref_position.clear()
        self.ref_velocities.clear()
        self.rmse_x_real.clear()
        self.rmse_y_real.clear()
        #Facciamo vedere solo una traiettoria alla volta
        self.real_path = Path()
        self.ref_path = Path()
        #Counter
        self.wait_count = 0.0
    
    
    def publish_reference_position(self,current_time: Time, x_ref: float, y_ref: float):
        current_ref_msg = PointStamped()
        current_ref_msg.header.stamp = current_time
        current_ref_msg.header.frame_id = 'odom' #Servirà per RViz2
        current_ref = Point()
        current_ref.x = x_ref
        current_ref.y = y_ref
        current_ref_msg.point = current_ref
        self.ref_signal_pub.publish(current_ref_msg)

    def publish_current_position(self, current_time: Time):
        current_xb_msg = PointStamped()
        current_xb_msg.header.stamp = current_time
        current_xb_msg.header.frame_id = 'odom' ##Servirà per RViz2
        current_xb = Point()
        current_xb.x = self.current_pose[0] + self.front_point * math.cos(self.current_pose[2])
        current_xb.y = self.current_pose[1] + self.front_point * math.sin(self.current_pose[2])
        current_xb_msg.point = current_xb
        self.xb_signal_pub.publish(current_xb_msg)
    
    def publish_reference_path(self, current_time: Time, x_ref: float, y_ref: float):
        pose_ref = PoseStamped()
        pose_ref.header.stamp = current_time
        pose_ref.header.frame_id = 'odom'
        #Posizione
        pose_ref.pose.position.x = x_ref
        pose_ref.pose.position.y = y_ref
        #Orientamento (quaternion, occorre per forza scrivere qualcosa altrimento si avrebbe un valore non valido e si verifica un errore)
        #In questo modo stiamo dicendo nessuna rotazione.
        pose_ref.pose.orientation.w = 1.0

        self.ref_path.poses.append(pose_ref)
        self.ref_path.header.stamp = current_time
        self.ref_path.header.frame_id = 'odom'

        self.path_ref_signal_pub.publish(self.ref_path)

    def publish_current_path(self, current_time: Time):
        pose_real = PoseStamped()
        pose_real.header.stamp = current_time
        pose_real.header.frame_id = "odom"

        #Posizione
        pose_real.pose.position.x = self.current_pose[0] + self.front_point * math.cos(self.current_pose[2])
        pose_real.pose.position.y = self.current_pose[1] + self.front_point * math.sin(self.current_pose[2])
        #Orientamento (quaternion, occorre per forza scrivere qualcosa altrimento si avrebbe un valore non valido e si verifica un errore)
        #In questo modo stiamo dicendo nessuna rotazione.
        pose_real.pose.orientation.w = 1.0

        self.real_path.poses.append(pose_real)
        self.real_path.header.stamp = current_time
        self.real_path.header.frame_id = "odom"
        
        self.path_xb_signal_pub.publish(self.real_path)
    
    def calculate_rmse(self):
        sum_errors = 0.0
        #self.get_logger().info(f"num1: {len(self.ref_position)}, num2: {len(self.rmse_x_real)}")
        for i in range(len(self.ref_position)):
            square_x_error = (self.rmse_x_real[i] - self.ref_position[i][0])**2
            square_y_error = (self.rmse_y_real[i] - self.ref_position[i][1])**2
            sum_errors += (square_x_error + square_y_error)
        #Calcoliamoci l'errore quadratico medio (MSE) [m]
        mse = sum_errors / len(self.ref_position)
        self.get_logger().info(f"MSE: {mse}")
        rmse = math.sqrt(mse)
        self.get_logger().info(f"RMSE: {rmse}")


#Main
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrotto da tastiera")
    finally:
        node.stop_node() #se è stata eseguita la traiettoria e abbiamo fatto lo shutdown() non verrà più eseguita
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#OSS: fino a quando non viene eseguito lo spin() il nodo non fa nulla,
#non vengono eseguiti callback, timer, etc.. (i vari pub verrano però eseguiti).
#Quindi eventuali timer, callback non verranno eseguiti fino a quando non si arriva allo spin()