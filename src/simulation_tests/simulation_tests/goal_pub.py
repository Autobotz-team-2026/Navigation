import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class GoalPublisher(Node):
    def __init__(self, x, y):
        super().__init__('pub_test')
        self.goal_x = float(x)
        self.goal_y = float(y)

        self.goalPub = self.create_publisher(Float32MultiArray, '/block_pose', 10)
        self.pubTimer = self.create_timer(1, self.pose_pub)

    def pose_pub(self):
        msg = Float32MultiArray()
        msg.data = [self.goal_x, self.goal_y]
        self.goalPub.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    x = input("Digite o valor de x: ")
    y = input("Digite o valor de y: ")
    node = GoalPublisher(x, y)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()