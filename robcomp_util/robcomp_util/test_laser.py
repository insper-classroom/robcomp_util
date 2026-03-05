import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.laser import Laser

class TestLaser(Node, Laser): # Mude o nome da classe
    def __init__(self):
        super().__init__('test_laser_node') # Mude o nome do nó
        Laser.__init__(self)
        self.opening = 10

        # Por fim, inicialize o timer
        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        print("self.front: ", self.front)
        print("Tem coisa perto? ", min(self.front) < 1.0)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = TestLaser() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()