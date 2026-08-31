#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import configparser
import os
from ament_index_python.packages import get_package_share_directory



class CentralController(Node):

    def __init__(self):
        super().__init__('central_controller') 
        self.config = configparser.ConfigParser()
        
        pacote_dir = get_package_share_directory('orion_bringup')
        caminho_cfg = os.path.join(pacote_dir, 'config', 'orion_params.cfg')
        
        if not os.path.exists(caminho_cfg):
            raise FileNotFoundError(f"Arquivo de configuracao nao encontrado em: {caminho_cfg}")
        self.config.read(caminho_cfg)


        self.task = self.config.get('task-params', 'task_type')
        self.general_state = "Starting"
        self.base_status = ""
        self.scara_status = ""
        self.command_sent = False

        self.n_blocks = self.config.getint('task-params', 'number_of_blocks')
        self.n_blocks_picked = 0

        self.base_confirmation_sub = self.create_subscription(String, '/base/confirmation', self.cb_base_confirmation, 10)
        self.scara_confirmation_sub = self.create_subscription(String, '/scara/confirmation', self.cb_scara_confirmation, 10)
        
        self.base_command_pub = self.create_publisher(String, '/base/command', 10)
        self.scara_command_pub = self.create_publisher(String, '/scara/command', 10)

        self.get_logger().info("///////////////////////////")
        self.get_logger().info("Central de comando Orion Pax")
        self.get_logger().info("///////////////////////////")
        self.get_logger().info(f"=== TASK CARREGADA: {self.task.upper()} ===")
        self.get_logger().info(f"Numero de blocos: {self.n_blocks}")
        self.get_logger().info(f"Modelo de cinematica: Diferencial/Skidsteer")
        
        input("\n>>> Pressione [ENTER] para iniciar o ciclo do CONTROLADOR... <<<\n")
        
        self.get_logger().info("Iniciando a Máquina de Estados!")

        self.controller_execute_cycle = self.create_timer(0.2, self.controller_execute)

    def cb_base_confirmation(self, msg: String):
        cmd = msg.data.strip()
        self.base_status = cmd

    def cb_scara_confirmation(self, msg: String):
        cmd = msg.data.strip()
        self.scara_status = cmd

    def update(self, command: String):
        print("------------------------------------")
        print(f"Excuting...: {command}")
        print(f"Base status: {self.base_status}")
        print(f"Scara status: {self.scara_status}")
        print(f"Blocks to operate status: {self.n_blocks_picked}/{self.n_blocks}")
        print("------------------------------------")

    def base_command_publisher(self, command: str):
        msg = String()
        msg.data = command
        self.base_command_pub.publish(msg)

    def scara_command_publisher(self, command: str):
        msg = String()
        msg.data = command
        self.scara_command_pub.publish(msg)

    def controller_execute(self):
        if self.task == "container-filler":
            self.execute_container_filler_controller()

    def execute_container_filler_controller(self):
        command = ""

        if self.n_blocks_picked < self.n_blocks:

            match self.general_state:

                case "Starting":
                    command = "Retract Arm"
                    if not self.command_sent:
                        self.scara_command_publisher(command)
                        self.command_sent = True

                    if self.scara_status == "Arm Retracted":
                        self.command_sent = False 
                        self.general_state = "Send robot to Pick"

                case "Send robot to Pick":
                    command = "Go to pick pose"
                    if not self.command_sent:
                        self.base_command_publisher(command)
                        self.command_sent = True

                    if self.base_status == "Arrived at pick pose":
                        self.command_sent = False
                        self.general_state = "Atempt to grab a block"

                case "Atempt to grab a block":
                    command = "Pick Block"
                    if not self.command_sent:
                        self.scara_command_publisher(command)
                        self.command_sent = True

                    if self.scara_status == "Block Picked":
                        self.command_sent = False
                        self.general_state = "Retrieve arm at pick"

                case "Retrieve arm at pick":
                    command = "Retract Arm"
                    if not self.command_sent:
                        self.scara_command_publisher(command)
                        self.command_sent = True

                    if self.scara_status == "Arm Retracted":
                        self.command_sent = False
                        self.general_state = "Send robot to Place"

                case "Send robot to Place":
                    command = "Go to place pose"
                    if not self.command_sent:
                        self.base_command_publisher(command)
                        self.command_sent = True

                    if self.base_status == "Arrived at place pose":
                        self.command_sent = False
                        self.general_state = "Atempt to place a block"

                case "Atempt to place a block":
                    command = "Place Block"
                    if not self.command_sent:
                        self.scara_command_publisher(command)
                        self.command_sent = True

                    if self.scara_status == "Block Placed":
                        self.command_sent = False
                        self.general_state = "Starting"
                        self.n_blocks_picked += 1
        else:
            command = "Go to finish pose"
            if not self.command_sent:
                self.base_command_publisher(command)
                self.command_sent = True
                
            if self.base_status == "Arrived at finish pose":
                self.general_state = "Task completed"

        self.update(command)
    


def main(args=None):
    rclpy.init(args=args)

    node = CentralController()

    rclpy.spin(node)
   
if __name__ == '__main__':
    main()
   