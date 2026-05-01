import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from time import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('ControllerNode avviato. Pubblicazione su /cmd_vel.')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Variabili per controllare il movimento ---
        self.vertical_velocity = 0.5 # Velocità di salita/discesa in m/s
        self.angular_velocity_z = 0.2 # Velocità di rotazione (yaw) in rad/s (opzionale)
        self.duration_per_phase = 3.0 # Durata di ogni fase del movimento (in secondi)

        self.start_time = time() # Tempo di avvio del nodo
        self.phase = 0 # 0: Salita, 1: Mantenimento, 2: Discesa, 3: Rotazione (opzionale)

        # Crea un timer che chiamerà la funzione timer_callback periodicamente.
        # Frequenza di pubblicazione: 20 Hz (ogni 0.05 secondi)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = time()
        elapsed_time = current_time - self.start_time

        twist_msg = Twist() # Crea un nuovo messaggio Twist vuoto

        # Logica di controllo per le fasi di movimento
        if self.phase == 0: # Fase di salita
            twist_msg.linear.z = self.vertical_velocity # Salita
            self.get_logger().info(f'Fase 0: Salita (linear.z={self.vertical_velocity:.2f})')
            if elapsed_time >= self.duration_per_phase:
                self.phase = 1 # Passa alla fase successiva
                self.start_time = current_time # Resetta il timer per la nuova fase
        elif self.phase == 1: # Fase di mantenimento (nessun movimento)
            twist_msg.linear.z = 0.0 # Fermo in verticale
            self.get_logger().info(f'Fase 1: Mantenimento (linear.z=0.0)')
            if elapsed_time >= self.duration_per_phase:
                self.phase = 2 # Passa alla fase successiva
                self.start_time = current_time
        elif self.phase == 2: # Fase di discesa
            twist_msg.linear.z = -self.vertical_velocity # Discesa
            self.get_logger().info(f'Fase 2: Discesa (linear.z={-self.vertical_velocity:.2f})')
            if elapsed_time >= self.duration_per_phase:
                self.phase = 0 # Torna all'inizio del ciclo (salita)
                self.start_time = current_time

        # Altri campi di Twist (linear.x, linear.y, angular.x, angular.y, angular.z)
        # Se non impostati, saranno 0.0 per default.
        # twist_msg.angular.z = self.angular_velocity_z # Esempio: aggiungere rotazione

        # Pubblica il messaggio Twist sul topic /cmd_vel
        self.publisher_.publish(twist_msg)

    def destroy_node(self):
        self.get_logger().info('ControllerNode spento.')
        # È buona pratica pubblicare una velocità di zero prima di spegnere
        # per fermare il drone immediatamente.
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args) # Inizializza la libreria ROS per Python
    node = ControllerNode() # Crea un'istanza del tuo nodo
    try:
        rclpy.spin(node) # Mantiene il nodo attivo ed esegue i callback in background
    except KeyboardInterrupt: # Permette di uscire dal nodo con Ctrl+C
        pass
    finally:
        node.destroy_node() # Assicurati che le risorse del nodo siano pulite
        rclpy.shutdown() # Chiude la libreria ROS

if __name__ == '__main__':
    main()