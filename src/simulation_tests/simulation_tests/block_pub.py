import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Vector3


class BlockPublisher(Node):
    def __init__(self):
        super().__init__('block_test')

        self.redBlock = None
        self.blueBlock = None


        self.redBlockSub = self.create_subscription(Pose, '/red/block/pose', self.redSetter, 10)
        self.blueBlockSub = self.create_subscription(Pose, '/blue/block/pose', self.blueSetter, 10)
        self.baseSub = self.create_subscription(PoseArray, '/robo_ground_truth/pose', self.correctedPose, 10)

        self.redPub = self.create_publisher(Vector3, '/red/pose', 10)
        self.bluePub = self.create_publisher(Vector3, '/blue/pose', 10)
        
    def redSetter(self, msg):
        self.redBlock = msg

    def blueSetter(self, msg):
        self.blueBlock = msg

    def correctedPose(self, msg):

        if self.redBlock is None or self.blueBlock is None:
            return
        
        if len(msg.poses) == 0:
            return
        
        robo_pose = msg.poses[0]
        rightX_red = self.redBlock.position.x - robo_pose.position.x
        rightY_red = self.redBlock.position.y - robo_pose.position.y
        rightZ_red = self.redBlock.position.z - robo_pose.position.z

        rightX_blue = self.blueBlock.position.x - robo_pose.position.x
        rightY_blue = self.blueBlock.position.y - robo_pose.position.y
        rightZ_blue = self.blueBlock.position.z - robo_pose.position.z

        rightRed = Vector3()

        rightBlue = Vector3()
        rightRed.x = rightX_red
        rightRed.y = rightY_red
        rightRed.z = rightZ_red

        rightBlue.x = rightX_blue
        rightBlue.y = rightY_blue
        rightBlue.z = rightZ_blue

        self.redPub.publish(rightRed)
        self.bluePub.publish(rightBlue)

        
def main(args = None):
    rclpy.init(args = args)
    node = BlockPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()