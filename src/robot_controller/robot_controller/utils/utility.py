import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from typing import Optional
import math

#Questo metodo permette di raccogliere il primo messaggio valido dal
#topic /turtle1/pose in modo da poter inizializzare le variabili.
def wait_for_message(node: Node, 
                     msg_type: type, #Nel nostro caso abbiamo due scelte: Odometry e Pose. Il valore type indica qualunque classe.
                     topic_name: str, 
                     hz: float,
                     timeout: Optional[float] = None): #Indica che il valore ricevuto può essere float oppure None
    
    #Lista per memorizzare i messaggi ricevuti dal topic
    msg_container = []

    #(Funzione locale), funzione di callback che salva il msg all'interno della lista
    def save_msg(msg):
        msg_container.append(msg)
    
    #Creiamo un subscriber temporaneo
    temporary_subscriber = node.create_subscription(msg_type, 
                                                    topic_name, 
                                                    save_msg,
                                                    10) #Qos depth=10 (dim queue), possiamo impostare (in futuro) anche delle politche QoS
    
    #Ogni nodo ROS2 ha un proprio orologio interno e lo si ottiene mediante il 
    #metodo get_clock(). Tramite il metodo now() otteniamo l'istante di tempo attuale,
    #ma questo metodo non restituisce un numero ma un oggetto di tipo Time.
    #Il tempo registrato può essere relativo al tempo reale del sistema oppure 
    #al tempo simulato (relativo ad un simulatore Gazebo).
    #Il metodo seconds_nanoseconds() restituisce una tupla di due valori interi:
    #il primo valore indica il numero intero di secondi mentre la seconda i nanosecondi
    #(parte frazionaria del secondo) (es. (120340, 789987))
    start_time = node.get_clock().now().seconds_nanoseconds()[0] #consideriamo i secondi
    while rclpy.ok(): #metodo che restituisce True o False se il sistema ROS2 è attivo o meno
        rclpy.spin_once(node=node, timeout_sec=1.0 / hz) #attiviamo lo spin relativo a questo nodo a tratti (durata pari a timeout_sec).
                                                         #dato che non abbiamo ancora creato gli altri pub,sub e timer non si hanno problemi.
                                                         #Per rendere il codice più robusto potrei inserire un try cathc in quanto può generare errore
        if msg_container: #True se la lista non è vuota
            node.destroy_subscription(temporary_subscriber)
            return msg_container[0]
        
        if timeout is not None:
            now = node.get_clock().now().seconds_nanoseconds()[0]
            #Tempo rimasto
            remaining_time = now - start_time
            if remaining_time > timeout:
                node.destroy_subscription(temporary_subscriber)
                return None
            
#SPIEGAZIONE TECNICA: quello che facciamo è andare a creare un subscriber che per 0.1 secondi si
#mette ad ascoltare sul topic di interesse. Se in questo lasso di tempo viene registrato qualcosa,
#verrà eseguita la callback associata, la quale aggiunge il messaggio (o più messagi se ne arrivano più in
#questo lasso di tempo) all'interno della lista.
#Se in questo lasso di tempo non è arivato nulla, lo spin si interrompe e si riprende
#la normale esecuzione del codice. Quindi quello che accade dopo è verificare se si hanno
#messaggi registrati, se ci sono possiamo tranquillamente distruggere questo 
#subscriber (non ci servirà più) e restituiamo il primo valore registrato.
#Se non si è registrato nulla si verifica il tempo trascorso. Se è minore del timeout
#non acccade nulla, si ripete nuovamente il ciclo for e quindi si farà nuovamente uno spin ...
#Se invece il timer è scaduto, distruggiamo comunque il subscriber e resituiamo None per 
#interrompere il ciclo. Questo verrà poi gestito dal processo chiamante.
    

#INFO:
#Quando facciamo ros2 run turtlesim turtlesim_node per esempio
#stiamo solo chiedendo di voler eseguire questo file all'interno di un 
#ambiente ROS2 (cerca nel ws la cartella install, apre l'entry point e avvia lo script).
#E' solo un comando di launcher
#Ma attenzione così facendo non è stato creto un nodo, a creare il nodo
#siamo noi tramite codice: rclpy.init(), node = MyNode(), rclpy.shutdown().
#Il sistema ROS2 (cioè in particolare il DDS) non è automaticamente attivo quando
#parte lo script. Facendo rclpy.init() inizializziamo però l'ambiente, ovvero:
#viene aperta la connessione con il DDS e si mette in allerta per esaudire varie richieste.
#Se creiamo un altro nodo occore sempre fare l'init(), perchè è come se stessimo dicendo
#all'ambiente ROS2, "ci sono anche io!"
#Tramite il metodo rclpy.spin(node) stiamo dicendo a ROS2 di gestire le callback relative
#a quel nodo passatogli come parametro. Si avvia un loop interno dove si ascolatano
#in maniera intelligente i vari topic, timer e si eseguono le callback associate.
#Fino a quando non viene invocata tale funzione, il nodo esiste ma non reagirebbe mai.

#Funzione in cui ottengo yaw (theta) dal quaternione
def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp) #espresso in radianti