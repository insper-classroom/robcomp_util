import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Andar(Node): # Mude o nome da classe
    def __init__(self): # Mude o nome do nó
        super().__init__('andar_node')
        self.timer = None

        self.robot_state = 'done' # Comece em 'done' - reset iniciará a ação
        self.state_machine = { # Adicione quantos estados forem necessários
            'andar': self.andar,
            'stop': self.stop,
            'done': self.done,
        }

        # Inicialização de variáveis
        self.v = 0.3

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def reset(self, distancia):
        self.twist = Twist()
        self.robot_state = 'andar' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.control) # Timer para o controle
        ### Iniciar variaveis da ação
        self.threshold = distancia / self.v
        self.tempo_inicial = self.get_clock().now()

    def andar(self):
        self.twist.linear.x = self.v

        self.dt = (self.get_clock().now() - self.tempo_inicial).to_msg()
        self.dt = self.dt.sec + self.dt.nanosec / 10**9
        
        if self.dt > self.threshold:
            self.robot_state = "stop"
            self.twist = Twist()

    def stop(self):
        self.twist = Twist() # Zera a velocidade
        print("Parando o robô.")
        self.timer.cancel() # Finaliza o timer
        self.timer = None # Reseta a variável do timer
        self.robot_state = 'done' # Ação finalizada
    
    def done(self):
        self.twist = Twist() # Zera a velocidade

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = Andar() # Cria o nó

    rclpy.spin_once(ros_node, timeout_sec=1) # Processa as callbacks uma vez
    print("oahsfpashfpohdspouhfasuoi")
    ros_node.reset(distancia = 1.0) # Reseta o nó para iniciar a ação

    while not ros_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
        rclpy.spin_once(ros_node, timeout_sec=1) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown()    # Finaliza o ROS2

if __name__ == '__main__': # Executa apenas se for o arquivo principal
    main()