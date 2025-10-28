import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
# Importar a classe da acao do arquivo, como por exemplo
from robcomp_util.goto import GoTo
# Adicione aqui os imports necessários

class Quadrado(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('quadrado_node') # Mude o nome do nó
        # Outra Herança que você queira fazer
        self.goto_node = GoTo() # Cria o nó da GoTo

        self.robot_state = 'client_goto'
        self.state_machine = {
            'client_goto': self.goto, # Estado para GERENCIAR a ação
            'done': self.done
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        ## Por fim, inicialize o timer
        self.timer = self.create_timer(0.1, self.control)
    

    def goto(self):
        print("\nIniciando movimento de ação...")
        if self.goto_node.timer is None:
            rclpy.spin_once(self.goto_node) # Processa as callbacks uma vez
            point = Point(x=0.0, y=1.2, z=0.0) # Define o ponto alvo
            self.goto_node.reset(point) # Reseta o nó para iniciar a ação

        rclpy.spin_once(self.goto_node) # Processa os callbacks e o timer

        if self.goto_node.robot_state == 'done':
            print("Ação de movimento finalizada.")
            self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        if self.robot_state != 'client_goto':
            self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = Quadrado() # Cria o nó

    while not ros_node.robot_state == 'done': # Enquanto o robô não estiver parado
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown() # Encerra o ROS2

if __name__ == '__main__':
    main()