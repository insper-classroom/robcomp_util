import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.odom import Odom

class TestOdomNode(Node, Odom): # Mude o nome da classe
    def __init__(self):
        super().__init__('node_name_here') # Mude o nome do nó
        Odom.__init__(self)
        
        # Por fim, inicialize o timer
        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        print("\n")
        print("self.x: ", self.x)
        print("self.y: ", self.y)
        print("self.yaw: ", self.yaw)

        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = TestOdomNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()