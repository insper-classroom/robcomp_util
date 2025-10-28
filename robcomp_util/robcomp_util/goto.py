import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from robcomp_util.odom import Odom

class GoTo(Node, Odom): # Mude o nome da classe

    def __init__(self, node = 'goto_node'): # Mude o nome do nó
        super().__init__(node)
        Odom.__init__(self)
        self.timer = None

        self.robot_state = 'done' # Comece em 'done' - reset iniciará a ação
        self.state_machine = { # Adicione quantos estados forem necessários
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop,
            'done': self.stop
        }

        # Inicialização de variáveis
        self.threshold_angle = np.deg2rad(1)
        self.threshold_distance = 0.1
        self.kp_ang = 0.5
        self.kp_lin = 0.5

        self.vel_max = 0.2
        self.min_abs_ang_vel = 0.02

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        # ...
    
    def reset(self, point):
        self.twist = Twist()
        self.robot_state = 'center' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.control) # Timer para o controle
        
        ### Iniciar variaveis da ação
        self.point = point
    
    def ajuste_angulo(self, angulo):
        """Ajusta o ângulo para o intervalo [-pi, pi]."""
        return np.arctan2(np.sin(angulo), np.cos(angulo))

    def get_angular_error(self):
        dx = self.point.x - self.x
        dy = self.point.y - self.y
        theta = np.arctan2(dy, dx)

        self.erro = self.ajuste_angulo(theta - self.yaw)
        self.d = np.sqrt(dx**2 + dy**2)
        print(f"Posicao: x = {self.x} y= {self.y} yaw = {np.rad2deg(self.yaw):.2f}")
        print(f'Erro angular: {np.rad2deg(self.erro):.2f} graus, Distância: {self.d:.2f} metros')
    
    def center(self):
        self.twist = Twist()
        if abs(self.erro) < self.threshold_angle:
            self.robot_state = 'goto'
            return
        self.twist.angular.z = self.kp_ang * self.erro
        self.twist.angular.z += abs(self.twist.angular.z)/self.twist.angular.z * self.min_abs_ang_vel


    def goto(self):
        self.twist = Twist()
        if abs(self.d) < self.threshold_distance:
            self.robot_state = 'stop'
            return
        self.twist.angular.z = self.kp_ang * self.erro
        self.twist.linear.x = min(self.kp_lin * self.d, self.vel_max)

    def stop(self):
        self.twist = Twist() # Zera a velocidade
        print("Parando o robô.")
        self.timer.cancel() # Finaliza o timer
        self.timer = None # Reseta a variável do timer
        self.robot_state = 'done' # Ação finalizada

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.get_angular_error()
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = GoTo() # Cria o nó

    rclpy.spin_once(ros_node) # Processa as callbacks uma vez
    point = Point(x=1.2, y=0.0, z=0.0) # Define o ponto alvo
    point = Point(x=0.0, y=1.2, z=0.0) # Define o ponto alvo
    ros_node.reset(point) # Reseta o nó para iniciar a ação

    while not ros_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown()    # Finaliza o ROS2

if __name__ == '__main__': # Executa apenas se for o arquivo principal
    main()