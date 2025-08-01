import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
from robcomp_util.odom import Odom
# from robcomp_util.amcl import AMCL

class GoTo(Node, Odom): # Mude o nome da classe
    def __init__(self, point: Point = Point()):
        Node.__init__(self, 'quadrado_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        # AMCL.__init__(self)

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi/180
        self.kp_linear = 0.6
        self.kp_angular = 0.6
        self.max_vel = 0.5

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        self.reset(point)
        self.timer = self.create_timer(0.25, self.control)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def reset(self, point):
        self.point = point
        self.twist = Twist()
        self.robot_state = 'center'
        self.distance = 0.0
        self.erro = 0.0
        self.goal_yaw = 0.0

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.01:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)

        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = GoTo(Point( x = -3., y = 0., z = 0.))

    while rclpy.ok():
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()