#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from robot_interfaces.srv import GetWaypoints
import yaml
import os


class WaypointServer(Node):
    def __init__(self):
        super().__init__('waypoint_server')

        #Dichiarazione/definizione del service
        self.srv = self.create_service(GetWaypoints,  #Tipo di service (file .srv definito)
                                       'get_waypoints', #Nome service
                                       self.get_waypoints_callback) #Callback associata
        #Log di informazione
        self.get_logger().info("il Service 'get_waypoints' è attivo e in attesa di richieste")

        #Dichiriamo un parametro ROS2 chiamato waypoint_file e gli assegniamo un valore di defualt (percorso relativo del file)
        #Tale valore può essere modificato per esempio quando lanciamo il nodo da CLI
        self.declare_parameter('waypoint_file', 'config/waypoints.yaml')
        #Otteniamo il valore attuale del parametro waypoint_file, ne estrae il valore
        #Il metodo get_parameter() resituisce un oggetto Paramter
        self.file_path = self.get_parameter('waypoint_file').value

        #Caricamento dei waypoint, li leggiamo dal file di configurazione se possibile
        self.waypoints = self.load_waypoints_from_file(self.file_path)

        #Se non ci sono waypoint nel file, passiamo alla modalità interattiva
        if not self.waypoints:
            self.get_logger().warn("Nessun file trovato oppure è vuoto. Passo alla modalità interattiva...")
            self.waypoints = self.ask_waypoints_interactively()
        else:
            self.get_logger().info(f"File trovato. Caricati {len(self.waypoints)} waypoint dal file.")

    #Andiamo a leggere il contenuto del file .yaml contente i waypoint
    def load_waypoints_from_file(self, path):
        if not os.path.exists(path): #Se il file non esiste restituiamo una lista vuota
            self.get_logger().warn(f"File {path} non trovato")
            return []
        try:
            #Proviamo ad aprire il file in modalità lettura.
            #Il costrutto with .. as garantisce che il file venga aperto e anche in caso di errore venga chiuso 
            with open(path, 'r') as file:
                data = yaml.safe_load(file) #usiamo il modulo yaml ed in particolare utilizziamo il metodo safe_load
                                            #che lo converte in un ogetto Python: dizionari, liste.. dipende dalla struttura
                if not data or 'waypoints' not in data: #Verifco se tale oggetto è vuoto oppure non contiene la parola waypoints
                    self.get_logger().warn("File YAML vuoto o malformato")
                    return []
                points = []
                for wp in data['waypoints']: #Prendo ogni valore
                    x_val = float(wp[0])
                    y_val = float(wp[1])
                    points.append(Point(x=x_val, y=y_val, z=0.0))
                return points
        except Exception as e:
            self.get_logger().error(f"Errore durante la lettura del file: {e}")
            return []

    #Inserimento manuale 
    def ask_waypoints_interactively(self):
        points = []
        self.get_logger().info("Inserisci i waypoint manualmente (scrivi 'fine' per terminare):")
        while True:
            line = input("Waypoint (x y): ") #Contiene l'input fatto dall'utente, voglio che inserisca due numeri separati da uno spazio
            if line.lower() in ['fine', 'end', 'exit']: 
                break
            try:
                x, y = map(float, line.split()) #Il metodo split() divide la stringa in una lista di pezzi usando come sepratore predefinito lo spazio
                #Per esempio "0.5 0.3" diventa ['0.5', '0.3']. Tramite il metodo map() si applica una funzione, in questo caso float, ad oni elemento della lista,
                #e poi facciamo una assegnazione multipla. 
                points.append(Point(x=x, y=y, z=0.0))
            except ValueError:
                print("Inserisci due numeri separati da spazio (es: 0.5 0.3)")
        return points

    #Callback del server
    def get_waypoints_callback(self, request, response):
        if not self.waypoints: #Se la lista dei waypoint è vuota (per esempio l'utente non vuole inserire nulla)
            response.success = False
            response.message = "Nessun waypoint disponibile"
            self.get_logger().error("Nessun waypoint da inviare")
            return response
        #Se invece abbiamo watpoint da inviare
        response.waypoints = self.waypoints
        response.success = True
        response.message = f"Inviati {len(self.waypoints)} waypoint"
        self.get_logger().info(f"Inviati {len(self.waypoints)} waypoint al client.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointServer()
    rclpy.spin(node) #Mettiamo in attesa
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
