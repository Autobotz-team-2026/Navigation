#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ScaraControl(Node):
    def __init__(self):
        super().__init__('cmd_pub')

        #Initials variables
        self.goal = [0.0, 0.0]
        self.cords_received = False

        #Publishers and Subscribers
        self.goalSub = self.create_subscription(Float32MultiArray, '/goal_pose', self.goal_pose_setter, 10)

    def goal_pose_setter(self, msg):
        if len(msg.data) >= 2:
            self.goal[0] = msg.data[0]
            self.goal[1] = msg.data[1]
            self.cords_received = True
            self.get_logger().info(f"Coordenadas recebidas: ({self.goal[0], self.goal[1]})")
        else:
            self.get_logger().info("Coordenadas não foram recebidas.")



def main(args = None):
    rclpy.init(args = args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()