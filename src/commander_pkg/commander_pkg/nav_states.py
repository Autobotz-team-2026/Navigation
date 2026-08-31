#!/usr/bin/env python3


import rclpy


from rclpy.node import Node


from rclpy.action import ActionClient


from rclpy.action.client import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose

INITIAL_POSE = (0.0, 0.0, 0.0, 1.0)
# tupla com a pose inicial do robô no mapa: (x, y, z_orientacao, w)
# x e y são as coordenadas no mapa
# z e w são o quaternion de orientação 

PICK_POSE = (5.911, 1.996, 0.0, 1.0)


PLACE_POSE = (4.087, 12.971, 0.0, 0.50)

FINISH_POSE = (3.55469, 6.77468, 0.0, 0.0)



class MobileBaseFSM(Node):


    def __init__(self):
        super().__init__('mobile_base_fsm')

        self.state = 'IDLE'
        # estado inicial

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # cria um publisher no tópico /initialpose
        #  tópico  lido pelo AMCL pra saber onde o robô começa no mapa
        

        self.confirmation_pub = self.create_publisher(
            String, '/base/confirmation', 10)
        # cria um publisher no tópico /base/confirmation
        # publica strings como "Arrived at pick pose" quando o robô chega no destino
        # a central escuta esse tópico pra saber que pode mandar o próximo comando (a principio no codigo)

        self.create_subscription(
            String, '/base/command', self.cb_command, 10)
        # se inscreve no tópico /base/command
        # a central publica strings tp "Go to pick pose"
        # quando chegar uma mensagem, o ROS2 chama cb_command automaticamente

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # cria o action client conectado ao servidor 'navigate_to_pose' do Nav2
        #meio complicado de usar mas da pra saber sobre confirmações
        self._init_timer = self.create_timer(2.0, self.publish_initial_pose)
        # cria um timer que dispara publish_initial_pose depois de 2 segundos  porque o AMCL demora um pouco pra subir após o launch
        # guardar a referência em self._init_timer pra poder cancelar depois

        self.get_logger().info(f'Estado: {self.state} — aguardando AMCL subir')
        # loga no terminal o estado atual

    def publish_initial_pose(self):
        # chamado pelo timer 2 segundos após o nó subir
        # publica a pose inicial no /initialpose pra o AMCL se localizar no mapa

        msg = PoseWithCovarianceStamped()
        # cria uma mensagem do tipo PoseWithCovarianceStamped (utilizada pelo AMCL)

        msg.header.frame_id = 'map'
        # define o frame de referência como 'map'
        # significa que as coordenadas são relativas ao mapa

        msg.header.stamp = self.get_clock().now().to_msg()
        # define o timestamp da mensagem com o tempo atual do nó pro ROS2 sincronizar
        msg.pose.pose.position.x = INITIAL_POSE[0]
        # define a coordenada x da posição inicial (0.844)

        msg.pose.pose.position.y = INITIAL_POSE[1]
        # define a coordenada y da posição inicial (2.03)

        msg.pose.pose.orientation.z = INITIAL_POSE[2]
        # define o componente z do quaternion de orientação 
        msg.pose.pose.orientation.w = INITIAL_POSE[3]
        # define o componente w do quaternion de orientaçã

        self.initial_pose_pub.publish(msg)
        # publica a mensagem no tópico /initialpose
        # o AMCL recebe e usa pra inicializar a localização do robô no mapa

        self.get_logger().info('Pose inicial publicada, aguardando comandos')

        self.state = 'WAITING'
        # muda o estado de IDLE pra WAITING
        # agora o nó está pronto pra receber comandos da central

        self._init_timer.cancel()
        # cancela o timer pra não publicar a pose inicial de novo

    def make_pose(self, x, y, z, w):
        # função auxiliar que monta um PoseStamped a partir de x, y, z, w

        pose = PoseStamped()

        pose.header.frame_id = 'map'

        pose.header.stamp = self.get_clock().now().to_msg()
        # mesma coisa, sincronizar

        pose.pose.position.x = x

        pose.pose.position.y = y

        pose.pose.position.z = 0.0
        # z da posição sempre 0  robô se move no plano 2D

        pose.pose.orientation.z = z

        pose.pose.orientation.w = w

        return pose
        # retorna o PoseStamped montado

    def cb_command(self, msg: String):
        # callback chamado toda vez que chega uma mensagem no /base/command
        # msg.data contém a string enviada pela central

        if self.state != 'WAITING':
            self.get_logger().warn(f'Comando ignorado, estado atual: {self.state}')
            return
        # se o robô não estiver em WAITING, ignora o comando para evitar mandar dois goals ao mesmo tempo
        # ex: se chegar "Go to place pose" enquanto o robô ainda tá indo pro pick

        cmd = msg.data.strip()
        # remove espaços e quebras de linha extras da string recebida
        # ex: "Go to pick pose\n" vira "Go to pick pose"
        # IA recomendou fazer isso para evitar erro

        if cmd == 'Go to pick pose':
            self.get_logger().info('Indo para pick pose')
            self.state = 'Go to pick'
            # muda estado pra 'Go to pick' antes de mandar o goal
            self.send_goal(self.make_pose(*PICK_POSE))
            # *PICK_POSE desempacota a tupla (1.0, 2.0, 0.0, 1.0), parece complicado mas eh equivalente a self.make_pose(1.0, 2.0, 0.0, 1.0)

        elif cmd == 'Go to place pose':
            self.get_logger().info('Indo para place pose')
            self.state = 'Go to place'
            self.send_goal(self.make_pose(*PLACE_POSE))

        elif cmd == 'Go to finish pose':
            self.get_logger().info('Indo para finish pose')
            self.state = 'Go to finish'
            self.send_goal(self.make_pose(*FINISH_POSE))

        else:
            self.get_logger().error(f'Comando desconhecido: "{cmd}"')
            # loga erro se chegar uma string que não é nenhum dos comandos esperados
            # estado permanece WAITING

    def send_goal(self, pose: PoseStamped):
        # monta e envia o goal pro action server do Nav2

        goal_msg = NavigateToPose.Goal()
        # cria o objeto goal do tipo NavigateToPose.Goal
        # esse é o formato que o Nav2 espera receber

        goal_msg.pose = pose
        # define a pose de destino dentro do goal

        self._nav_client.wait_for_server()
        # bloqueia até o action server do Nav2 estar disponível
        # necessário porque o Nav2 pode demorar pra subir após o launch

        future = self._nav_client.send_goal_async(goal_msg)
        # envia o goal de forma assíncrona  sem travar o nó
        # retorna um Future imediatamente, sem esperar resposta

        future.add_done_callback(self.cb_goal_accepted)
        # registra cb_goal_accepted como callback do future
        # quando o Nav2 responder se aceitou ou rejeitou o goal,
        # o future fica resolvido e cb_goal_accepted é chamado automaticamente

    def cb_goal_accepted(self, future):
        # chamado quando o Nav2 responde se aceitou ou rejeitou o goal

        goal_handle = future.result()
        # pega o goal_handle do future
        # goal_handle representa o goal em execução
        # permite cancelar, monitorar e pegar o resultado

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeitado pelo Nav2')
            self.state = 'WAITING'
            return
        # se o Nav2 rejeitou (pose fora do mapa, caminho impossível, etc)
        # volta pra WAITING pra aceitar um novo comando

        result_future = goal_handle.get_result_async()
        # pede pro goal_handle um Future do resultado final
        # esse Future só fica resolvido quando o robô chegar ou falhar

        result_future.add_done_callback(self.cb_goal_result)
        # registra cb_goal_result como callback
        # quando a navegação terminar, cb_goal_result é chamado automaticamente

    def cb_goal_result(self, future):
        # chamado quando a navegação termina — sucesso ou falha

        status = future.result().status
        # pega o status final da navegação
        # STATUS_SUCCEEDED, STATUS_ABORTED ou STATUS_CANCELED

        if status == GoalStatus.STATUS_SUCCEEDED:
            # navegação concluída com sucesso — robô chegou no destino

            msg = String()
            # cria mensagem de confirmação

            if self.state == 'Go to pick':
                self.get_logger().info('Arrived at pick pose')
                msg.data = 'Arrived at pick pose'
                # define a string de confirmação correspondente ao estado

            elif self.state == 'Go to place':
                self.get_logger().info('Arrived at place pose')
                msg.data = 'Arrived at place pose'

            elif self.state == 'Go to finish':
                self.get_logger().info('Arrived at finish pose')
                msg.data = 'Arrived at finish pose'

            self.confirmation_pub.publish(msg)
            # publica a confirmação no /base/confirmation
            # a central recebe e sabe que pode mandar o próximo comando

            self.state = 'WAITING'
            # volta pra WAITING 

        else:
            self.get_logger().warn(f'Goal falhou com status {status}')
            self.state = 'WAITING'
            # navegação falhou ,volta pra WAITING sem publicar confirmação
            # a central não recebe confirmação e pode reenviar o comando


def main(args=None):
    rclpy.init(args=args)


    node = MobileBaseFSM()

    rclpy.spin(node)
   
if __name__ == '__main__':
    main()
   