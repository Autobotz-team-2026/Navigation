import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math

class SensorTest(Node):
    def __init__(self):
        super().__init__('sub_test')

        self.imu_base = {'roll':0.0, 'pitch': 0.0, 'yaw':0.0}
        self.imu_arm1 = {'roll':0.0, 'pitch': 0.0, 'yaw':0.0}
        self.imu_arm2 = {'roll':0.0, 'pitch': 0.0, 'yaw':0.0}
        self.imu_gripper = {'roll':0.0, 'pitch': 0.0, 'yaw':0.0}

        #Sensors subscribers:
        self.imuBaseSub = self.create_subscription(Imu, '/imu_base', self.imuBaseSetter, 10)
        self.imuArm1Sub = self.create_subscription(Imu, '/imu_arm1', self.imuArm1Setter, 10)
        self.imuArm2Sub = self.create_subscription(Imu, '/imu_arm2', self.imuArm2Setter, 10)
        self.imuGripperSub = self.create_subscription(Imu, '/imu_gripper', self.imuGripperSetter, 10)
        self.imusPub = self.create_publisher(Float32MultiArray, '/imus_yaw', 10)

        self.showImusTimer = self.create_timer(0.02, self.showImus)

    def showImus(self):
        self.get_logger().info(f"Base IMU:{self.imu_base}")
        self.get_logger().info(f"Arm1 IMU:{self.imu_arm1}")
        self.get_logger().info(f"Arm2 IMU:{self.imu_arm2}")
        self.get_logger().info(f"Gripper IMU:{self.imu_gripper}")
        msg = Float32MultiArray()
        msg.data = [math.radians(self.imu_base["yaw"]), math.radians(self.imu_arm1["yaw"]), math.radians(self.imu_arm2["yaw"]), math.radians(self.imu_gripper["yaw"])]
        self.imusPub.publish(msg)


    def imuBaseSetter(self, msg):
        rpy = quaternionToEuler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.imu_base = {'roll':rpy[0], 'pitch':rpy[1], 'yaw':rpy[2]}
    
    def imuArm1Setter(self, msg):
        rpy = quaternionToEuler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.imu_arm1 = {'roll':rpy[0], 'pitch':rpy[1], 'yaw':rpy[2]-self.imu_base['yaw']}



    def imuArm2Setter(self, msg):
        rpy = quaternionToEuler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.imu_arm2 = {'roll':rpy[0], 'pitch':rpy[1], 'yaw':rpy[2]-self.imu_arm1['yaw']-self.imu_base['yaw']}


    def imuGripperSetter(self, msg):
        rpy = quaternionToEuler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.imu_gripper = {'roll':rpy[0], 'pitch':rpy[1], 'yaw':rpy[2]-self.imu_arm2['yaw']-self.imu_arm1['yaw']-self.imu_base['yaw']}



def quaternionToEuler(x,y,z,w):
    roll = math.atan2(2*(w*x+y*z), 1 - 2*(x**2+y**2))
    sin_pitch = 2 * (w * y - z * x)
    if sin_pitch > 1.0:
        sin_pitch = 1.0
    elif sin_pitch < -1.0:
        sin_pitch = -1.0
        
    pitch = math.asin(sin_pitch)

    # Yaw (eixo Z)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    rpy = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
    return rpy


def main(args = None):
    rclpy.init(args = args)
    node = SensorTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()