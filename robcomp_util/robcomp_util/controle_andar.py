import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
# Importar a classe da acao do arquivo, como por exemplo
from robcomp_util.andar import Andar
# Adicione aqui os imports necessários

class ControleAndar(Node): # Mude o nome da classe
    def __init__(self):
        super().__init__('controle_andar_node') # Mude o nome do nó
        # Outra Herança que você queira fazer
        self.andar_node = Andar() # Cria o nó da andar

        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar, # Estado para GERENCIAR a ação
            'done': self.done
        }

        self.estados_clientes = ['andar'] # Coloque aqui os estados que são "cliente de ação".

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        ## Por fim, inicialize o timer
        self.timer = self.create_timer(0.1, self.control)

    def andar(self):
        if self.andar_node.robot_state == 'done': # Se a ação NÂO FOI INICIADA
            print("\nIniciando [andar]...")
            rclpy.spin_once(self.andar_node, timeout_sec=1) # Processa as callbacks uma vez
            self.andar_node.reset(distancia=1) # Reseta o nó para iniciar a ação

        rclpy.spin_once(self.andar_node, timeout_sec=1) # Processa os callbacks e o timer
        self.dist = self.andar_node.v * self.andar_node.dt
        print("Distancia andada: ", self.dist)

        if self.andar_node.robot_state == 'done': # Se a ação FOI FINALIZADA
            self.andar_node.control() # Garante que o robo é parado antes de finalizar a ação
            print("[andar] Finalizada.")
            self.robot_state = 'done' # Muda para o próximo estado da máquina de estados

    def done(self):
        self.twist = Twist()

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        if self.robot_state not in self.estados_clientes: # Se o estado atual não é um estado "cliente de ação"
            self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = ControleAndar() # Cria o nó

    while not ros_node.robot_state == 'done': # Enquanto o robô não estiver parado
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown() # Encerra o ROS2

if __name__ == '__main__':
    main()