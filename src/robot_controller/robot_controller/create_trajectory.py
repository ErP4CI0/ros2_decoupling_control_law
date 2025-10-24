#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_controller.utils.utility import wait_for_message 
from robot_controller.utils.utility import yaw_from_quaternion
from robot_interfaces.msg import Trajectory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from robot_controller.trajectories.straight import Straight
from robot_controller.trajectories.sinusoid import Sinusoid
from robot_controller.trajectories.circle import Circle
from robot_controller.trajectories.eight import Eight
from robot_controller.trajectories.square import Square
from robot_controller.trajectories.waypoint_linear import WaypointLinear
from robot_controller.trajectories.waypoint_smooth import WaypointSmooth
import math
import numpy as np
from robot_interfaces.srv import GetWaypoints

class CreateTrajectory(Node):
    def __init__(self):
        super().__init__('create_trajectory_node')

        #Publisher
        self.trajectory_pub = self.create_publisher(Trajectory, '/reference_trajectory', 10)

        #Frequenza
        self.hz = 20.0 #[Hz] (deve essere corente con quella del controllore)
        #Punto di controllo anteriore
        self.front_point = 0.10 #[m]

        #Matrice di rotazione
        self.rotation_matrix = np.zeros((2,2), dtype = float)
        #Anch'essa è vuota inizialmente ed è un array numpy (in particolare è una matrice).
        #La setteremo nel momento in cui leggeremo lo yaw iniziale
        
        #Liste contenente i vari riferimenti da passare al controllore
        self.ref_position = []
        self.ref_velocities = []

        #Anidamo a creare il client che va a fare una richiesta la server
        self.client = self.create_client(GetWaypoints, 'get_waypoints')
        #Attendiamo che il server sia disponibile
        self.wait_for_service_availability(timeout_sec=20.0)
        #Eseguo la richiesta (subito all'avvio del nodo)
        self.waypoints = self.request_waypoints(timeout_sec=5.0)

        #Scelta traiettoria
        self.trajectory_type = input("Inserisci una traiettoria [retta, sinusoide, cerchio, otto, quadrato, spezzata, smooth]: ").strip().lower()
        #Il metodo strip() toglie eventuali spazi iniziali e finali, mentre
        #lower() rende tutti i caratteri minuscoli

        #Creazione della traiettoria di riferimento
        self.compute_trajectory()
        #Invio
        self.create_and_send_msg()
    
    #Attendiamo/Verifichiamo che il server sia disponibile
    def wait_for_service_availability(self, timeout_sec=20.0):
        self.get_logger().info("Attesa che il service 'get_waypoints' sia disponibile...")
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Service 'get_waypoints' non disponibile entro il timeout previsto.")
            self.destroy_node()
            rclpy.shutdown()
            exit(1) #Forziamo l'uscita, 1 significa che si ha avuto un errore
        self.get_logger().info("Service 'get_waypoints' disponibile")
    
    #Funzione per richiedere i waypoint al service:
    #Effettua una richiesta al service /get_waypoints e attende la risposta.
    #Se non arriva entro 'timeout_sec', o se la risposta non è valida,
    #viene restituita una lista di waypoint di default.
    def request_waypoints(self, timeout_sec=5.0):
        #Creiamo la richiesta (vuota, poiché la Request non prevede campi)
        req = GetWaypoints.Request()

        self.get_logger().info("Invio della richiesta dei waypoint al server")

        #Eseguiamo la chiamata in modo asincrono
        future = self.client.call_async(req)

        #Restiamo in attesa della risposta con un timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        #Nessuna risposta entro il timeout
        if future.result() is None:
            self.get_logger().error("Timeout scaduto: nessuna risposta ricevuta dal service.")
            return self.get_default_waypoints()

        #Risposta ricevuta ma non valida
        response = future.result()
        if not response.success or len(response.waypoints) == 0:
            self.get_logger().warn(f"Il Service ha risposto ma senza waypoint validi ({response.message}).")
            return self.get_default_waypoints()

        #Risposta corretta
        waypoint_list = []
        for point in response.waypoints:
            waypoint_list.append([point.x, point.y])
        self.get_logger().info(f"Ricevuti {len(waypoint_list)} waypoint dal server con successo")
        return waypoint_list

    #Waypoint di default in caso di errore o assenza di risposta
    def get_default_waypoints(self):
        self.get_logger().info("Uso di waypoint di default.")
        return [
            [0.0, 0.0],
            [2.0, 0.0],
            [3.0, 2.0],
            [4.0, 2.5]
        ]

    def compute_trajectory(self):
        #Inizializziamo le variabili che utilizzeremo per costruire la traiettoria desiderata
        init_offset = self.set_initial_variables()
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

        elif self.trajectory_type == 'spezzata':
            waypoint_linear = WaypointLinear(hz=self.hz,
                                             rotation_matrix=self.rotation_matrix,
                                             init_offset=init_offset,
                                             waypoints=self.waypoints)
            self.ref_position, self.ref_velocities = waypoint_linear.to_global_frame()

        elif self.trajectory_type == 'smooth':
            waypoint_smooth = WaypointSmooth(hz=self.hz,
                                             rotation_matrix=self.rotation_matrix,
                                             init_offset=init_offset,
                                             waypoints=self.waypoints)
            self.ref_position, self.ref_velocities = waypoint_smooth.to_global_frame()
            #self.get_logger().info(f"Parametri usati:{waypoint_smooth.stamp()}")
        
        else:
            self.get_logger().error("Traiettoria non supportata")
            self.destroy_node()
            rclpy.shutdown() #Ferma tutti i nodi attivi e libera le risorse (socket, connessioni DDS, memoria)
            return
        
        self.get_logger().info(f"Traiettoria '{self.trajectory_type}' generata correttamente")
    
    
    #Inizializzazione della traiettoria
    def set_initial_variables(self):
        #Topic name e timeout
        topic_name = '/odom'
        timeout = 10.0 #[s]
        #Catturiamo il primo messaggio valido
        init_msg = wait_for_message(node=self, 
                                    msg_type=Odometry, 
                                    topic_name=topic_name,
                                    hz=self.hz, 
                                    timeout=timeout)
        #Gestione della casistica in cui venga restituito None
        if init_msg is None:
            self.get_logger().error(f"Nessun messaggio registrato dal topic {topic_name} in {timeout} secondi")
            self.destroy_node() #Distruggiamo il nodo
            rclpy.shutdown() #Termina l'esecuzione dello spin ed eseguirà il blocco finally
            return #interrompiamo l'esecuzione di questo metodo altrimenti proseguirebbe sotto

        #Prendiamo il qauternione e ne calcoliamo lo yaw attuale, qui il msg Odometry è sofisticato, guarda tu stesso
        quaternion = init_msg.pose.pose.orientation
        init_theta = yaw_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        #Calcoliamo la posizione iniziale del punto anteriore attuale
        init_x = init_msg.pose.pose.position.x + self.front_point * math.cos(init_theta)
        init_y = init_msg.pose.pose.position.y + self.front_point * math.sin(init_theta)

        #Creiamo la matrice di rotazione
        self.rotation_matrix = np.array([
            [math.cos(init_theta), -math.sin(init_theta)],
            [math.sin(init_theta),  math.cos(init_theta)]
        ], dtype=float)

        return np.array([init_x, init_y], dtype=float)
    
    def create_and_send_msg(self):
        #Creiamo il messaggio ROS
        traj_msg = Trajectory()
        for pos, vel in zip(self.ref_position, self.ref_velocities):
            position = Point(x=pos[0], y=pos[1], z=0.0)
            velocity = Vector3(x=vel[0], y=vel[1], z=0.0)
            traj_msg.positions.append(position)
            traj_msg.velocities.append(velocity)
        #Pubblichiamo 
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info("Traiettoria pubblicata su /reference_trajectory")
    
    def regenerate_trajectory(self):
        self.trajectory_type = input("Inserisci una traiettoria [retta, sinusoide, cerchio, otto, quadrato, spezzata, smooth]: ").strip().lower()
        self.compute_trajectory()
        self.create_and_send_msg()


def main(args=None):
    rclpy.init(args=args)
    node = CreateTrajectory()
    try:
        while rclpy.ok():
            #rclpy.spin_once(node) #Inutile in questo caso, ma in caso di sviluppi futuri..
            new_traj = input("Vuoi creare una nuova traiettoria? [yes/no]: ").strip().lower()
            if new_traj == 'yes':
                node.regenerate_trajectory()
            else:
                node.get_logger().info("Chiusura del nodo create_trajectory")
                break #viene interrotto il ciclo
    except KeyboardInterrupt:
        node.get_logger().info("Interrotto da tastiera")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        exit(0)



if __name__ == '__main__':
    main()
