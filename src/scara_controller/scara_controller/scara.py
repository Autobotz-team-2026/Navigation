#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64, Float32, String, Float32MultiArray
import time


class JointPID:
    def __init__(self, kp, ki, kd, tol, goal):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.goal = goal
        self.prev_error = 0
        self.integral = 0.0
        self.last_time = None
        self.confirmation = False
        self.tolerance = tol

    def calculate(self, current_pos, current_time_sec):
        if self.last_time is None:
            self.last_time = current_time_sec
            return 0.0

        dt = current_time_sec - self.last_time
        if dt <= 0: return 0.0

        error = self.goal - current_pos

        self.integral += error * dt
        #self.integral = max(min(self.integral, 1.0), -1.0)
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.last_time = current_time_sec

        if abs(error) <= self.tolerance:
            self.confirmation = True
        else: 
            self.confirmation = False

        return output 

class ScaraControl(Node):
    def __init__(self):
        super().__init__('cmd_pub')
        self.get_logger().info("Scara Node started.", once=True)

        #Manipulator lengths.
        self.l1 = 0.425     #First arm
        self.l2 = 0.34      #Second arm

        #Initials variables.
        self.goal = [0.775, 0.0]                   #Final goal to manipulator.
        self.block_pose = [0.0, 0.0]
        self.theta1 = 0.0                        #First joint rotation.
        self.theta2 = 0.0                        #Second joint rotation.
        self.height = 0.09
        self.manipulator_state = "Stand By"      #Manipulator's state.
        self.block_received = False
        self.pid1 = JointPID(5, 0.5, 1, 0.005, self.theta1)
        self.pid2 = JointPID(5, 0.5, 1, 0.005, self.theta2)
        self.heightPid = JointPID(500, 10, 80, 0.07, self.height)

        #current positions:
        self.current_theta1 = 0.0
        self.current_theta2 = 0.0
        self.current_height = 0.0

        #Publishers and Subscribers
        self.stateCmdSub = self.create_subscription(String, '/scara/command', self.state_setter, 10) #Take the string from "/scara/command" and set the manipulator's state: "Retract", "Pick Block" or "Place Block".
        self.goalSub = self.create_subscription(Float32MultiArray, '/block_pose', self.block_pose_setter, 10) #Take the goal from "/goal_pose" and set the variables.
        
        self.imusSub = self.create_subscription(Float32MultiArray, '/imus_yaw', self.currentArmSetter, 10)
        self.heightSub = self.create_subscription(Float32, '/scara/height_sensor', self.currentHeightSetter, 10)
        
        self.loopTimer = self.create_timer(0.02, self.mainControl)

        self.arm1Pub = self.create_publisher(Float64, '/scara_arm_joint1/cmd_pos', 10) 
        self.arm2Pub = self.create_publisher(Float64, '/scara_arm_joint2/cmd_pos', 10)
        self.leftGripperPub = self.create_publisher(Float64, '/scara_left_gripper/cmd_pos', 10)
        self.rightGripperPub = self.create_publisher(Float64, '/scara_right_gripper/cmd_pos', 10)
        self.heightPub = self.create_publisher(Float64, '/scara_height_joint0/cmd_pos', 10)
        self.clawRotPub = self.create_publisher(Float64, '/scara_claw_rotation_joint0/cmd_pos', 10)
        self.confirmationPub = self.create_publisher(String, '/scara/confirmation', 10)

    def mainControl(self):

        confirmMsg = String()
        if self.manipulator_state == "Retract Arm":
            self.height = 0.05
            if self.heightPid.confirmation: 
                self.goal[0] = -0.775
                self.goal[1] = 0
                if self.pid1.confirmation and self.pid2.confirmation:
                    velRight = Float64()
                    velRight.data = 5.0
                    velLeft = Float64()
                    velLeft.data = -5.0
                    time.sleep(2)
                    self.rightGripperPub.publish(velRight)
                    self.leftGripperPub.publish(velLeft)
                    confirmMsg.data = "Arm Retracted"
                    self.confirmationPub.publish(confirmMsg)
                    self.manipulator_state = "Stand By"


        elif self.manipulator_state == "Place Block":
            self.goal[0] = 0.775
            self.goal[1] = 0
            if self.pid1.confirmation and self.pid2.confirmation:
                self.height = 0.35
                if self.heightPid.confirmation:
                    velRight = Float64()
                    velRight.data = -5.0
                    velLeft = Float64()
                    velLeft.data = 5.0
                    time.sleep(2)
                    self.rightGripperPub.publish(velRight)
                    self.leftGripperPub.publish(velLeft)
                    confirmMsg.data = "Block Placed"
                    self.confirmationPub.publish(confirmMsg)
                    self.manipulator_state = "Stand By"

        elif self.manipulator_state == "Pick Block" and self.block_received:
            self.goal[0] = self.block_pose[0]
            self.goal[1] = self.block_pose[1]
            if self.pid1.confirmation and self.pid2.confirmation:
                self.height = 0.35
                if self.heightPid.confirmation:
                    velRight = Float64()
                    velRight.data = 5.0
                    velLeft = Float64()
                    velLeft.data = -5.0
                    time.sleep(2)
                    self.rightGripperPub.publish(velRight)
                    self.leftGripperPub.publish(velLeft)
                    confirmMsg.data = "Block Picked"
                    self.confirmationPub.publish(confirmMsg)
                    self.manipulator_state = "Stand By"

        elif self.manipulator_state == "Stand By":
            pass
        
        elif self.manipulator_state == "Pick Block" and (not self.block_received):
            self.get_logger().info("Mising block's coordinates", once=True)
        
        #Calculating theta2:
        cos_theta2 = (self.goal[0]**2 + self.goal[1]**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        self.theta2 = math.acos(cos_theta2)

        #Calculating theta1:
        self.theta1 = math.atan2(self.goal[1], self.goal[0]) - math.atan2(self.l2*math.sin(self.theta2), self.l1 + self.l2*cos_theta2)

        self.pid1.goal = self.theta1
        self.pid2.goal = self.theta2
        self.heightPid.goal = self.height
        now_sec = self.get_clock().now().nanoseconds / 1e9
    

        v_control1 = self.pid1.calculate(self.current_theta1, now_sec)
        v_control2 = self.pid2.calculate(self.current_theta2, now_sec)
        v_controlHeight = -self.heightPid.calculate(self.current_height, now_sec)

        cmd1_msg = Float64()
        cmd1_msg.data = v_control1
        cmd2_msg = Float64()
        cmd2_msg.data = v_control2
        cmdHeight = Float64()
        cmdHeight.data = v_controlHeight
        self.heightPub.publish(cmdHeight)
        self.arm1Pub.publish(cmd1_msg)
        self.arm2Pub.publish(cmd2_msg)
        
        self.get_logger().info(f"Arm1 Error: {self.pid1.prev_error}", throttle_duration_sec = 2)
        self.get_logger().info(f"Arm2 Error: {self.pid2.prev_error}",throttle_duration_sec = 2)
        self.get_logger().info(f"Height error: {self.heightPid.prev_error}",throttle_duration_sec = 2)
 
        
    def currentHeightSetter(self, msg):
        self.current_height = msg.data

    def currentArmSetter(self, msg):
        self.current_theta1 = msg.data[1]
        self.current_theta2 = msg.data[2]                   

    def block_pose_setter(self, msg):
        self.block_pose[0] = msg.data[0]
        self.block_pose[1] = msg.data[1]
        self.block_received = True
        self.get_logger().info("Block received", once=True)
             
    def state_setter(self, msg):
        self.pid1.confirmation = False   ###MSG para trocar de estado deve ser enviado apenas UMA vez.
        self.pid2.confirmation = False
        self.heightPid.confirmation = False
        if msg.data == "Retract":
            self.manipulator_state = "Retract Arm"
            self.get_logger().info("Retract state setted", once=True)
        elif msg.data == "Pick Block" and self.block_received:
            velRight = Float64()
            velRight.data = -5.0
            velLeft = Float64()
            velLeft.data = 5.0 
            self.rightGripperPub.publish(velRight)
            self.leftGripperPub.publish(velLeft)
            self.manipulator_state = "Pick Block"
            self.get_logger().info("Pick block state setted", once=True)
        
        elif msg.data == "Pick Block" and not self.block_received:
            self.get_logger().info("Missing block cordinates.")

        elif msg.data == "Place Block":
            self.manipulator_state = "Place Block"  
            self.get_logger().info("Placing block state setted", once = True)
      

def main(args = None):
    rclpy.init(args = args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()