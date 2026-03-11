#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import math

class ScaraControl(Node):
    def __init__(self):
        super().__init__('cmd_pub')

        #Manipulator datas:
        self.l1 = 0.425
        self.l2 = 0.35
    

        #Initials variables
        self.goal = [0.0, 0.0]
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.cords_received = False

        #Publishers and Subscribers
        self.goalSub = self.create_subscription(Float32MultiArray, '/goal_pose', self.goal_pose_setter, 10)
        self.arm1Pub = self.create_publisher(Float64, '/scara_arm_joint1/cmd_pos', 10)
        self.arm2Pub = self.create_publisher(Float64, '/scara_arm_joint2/cmd_pos', 10)
        self.heightPub = self.create_publisher(Float64, '/scara_height_joint0/cmd_pos', 10)
        self.clawRotPub = self.create_publisher(Float64, '/scara_claw_rotation_joint0/cmd_pos', 10)

        self.timerCinematic = self.create_timer(0.5, self.thetasCalc)


    def goal_pose_setter(self, msg):
        if len(msg.data) >= 2:
            self.goal[0] = msg.data[0]
            self.goal[1] = msg.data[1]
            self.cords_received = True
            self.get_logger().info(f"Coordenadas recebidas: ({self.goal[0], self.goal[1]})")
        else:
            self.get_logger().info("Coordenadas não foram recebidas devidamente.")

    def thetasCalc(self):
        if self.cords_received:
            self.get_logger().info("Iniciando o cálculo da cinemática inversa...")
            #Calculating theta2:
            cos_theta2 = (self.goal[0]**2 + self.goal[1]**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
            cos_theta2 = max(-1.0, min(1.0, cos_theta2))
            self.theta2 = math.acos(cos_theta2)

            #Calculating theta1:
            self.theta1 = math.atan2(self.goal[1], self.goal[0]) - math.atan2(self.l2*math.sin(self.theta2), self.l1 + self.l2*cos_theta2)

            cmdJoint1 = Float64()
            cmdJoint1.data = self.theta1
            cmdJoint2 = Float64()
            cmdJoint2.data = self.theta2
            self.arm1Pub.publish(cmdJoint1)
            self.arm2Pub.publish(cmdJoint2)



def main(args = None):
    rclpy.init(args = args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()