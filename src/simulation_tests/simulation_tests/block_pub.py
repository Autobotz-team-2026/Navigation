import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from math import sqrt



class BlockPublisher(Node):
    def __init__(self):
        super().__init__('block_test')

        self.redBlock = None
        self.blueBlock = None


        self.redBlockSub = self.create_subscription(Pose, '/red/block/pose', self.redSetter, 10)
        self.blueBlockSub = self.create_subscription(Pose, '/blue/block/pose', self.blueSetter, 10)
        self.baseSub = self.create_subscription(PoseArray, '/robo_ground_truth/pose', self.correctedPose, 10)

        self.goalPub = self.create_publisher(Float32MultiArray, '/block_pose', 10)

        
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
        rightX_red = self.redBlock.position.x - robo_pose.position.x - 0.275
        rightY_red = self.redBlock.position.y - robo_pose.position.y
        rightZ_red = self.redBlock.position.z - robo_pose.position.z

        rightX_blue = self.blueBlock.position.x - robo_pose.position.x - 0.275
        rightY_blue = self.blueBlock.position.y - robo_pose.position.y
        rightZ_blue = self.blueBlock.position.z - robo_pose.position.z

        distBlue = sqrt(rightX_blue**2 + rightY_blue**2)
        distRed = sqrt(rightX_red**2 + rightY_red**2)

        if (distBlue and distRed) > 0.775:
            return
        
        if (distBlue <= distRed):
            pose = Float32MultiArray()
            pose.data = [rightX_blue, rightY_blue]
            self.goalPub.publish(pose)

        elif (distRed < distBlue):
            pose = Float32MultiArray()
            pose.data = [rightX_red, rightY_red] #Inverted
            self.goalPub.publish(pose)


        
def main(args = None):
    rclpy.init(args = args)
    node = BlockPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()