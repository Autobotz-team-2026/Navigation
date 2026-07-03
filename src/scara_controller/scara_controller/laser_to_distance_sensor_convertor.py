import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import statistics

class LaserToDistance(Node):
    def __init__(self):
        super().__init__('laser_to_distance')
        
        self.subscription = self.create_subscription(
            LaserScan, '/distance_scan', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/scara/height_sensor', 10)

        # Configurações do Filtro
        self.window_size = 5          # Tamanho da janela para mediana
        self.data_history = []        # Histórico de leituras
        self.alpha = 0.3              # Fator de suavização (0.0 a 1.0)
        self.last_filtered_value = 0.0
        self.first_run = True

        self.get_logger().info('Filtro Suavizador Orion iniciado.')

    def listener_callback(self, msg):
        # 1. Filtro de Range Básico
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            raw_min = min(valid_ranges)
            
            # 2. Filtro de Mediana (remove picos isolados)
            self.data_history.append(raw_min)
            if len(self.data_history) > self.window_size:
                self.data_history.pop(0)
            
            median_value = statistics.median(self.data_history)

            # 3. Filtro de Média Exponencial (EMA - suaviza a curva)
            if self.first_run:
                self.last_filtered_value = median_value
                self.first_run = False
            
            filtered_value = (self.alpha * median_value) + ((1.0 - self.alpha) * self.last_filtered_value)
            self.last_filtered_value = filtered_value

            # Publicação
            height_msg = Float32()
            height_msg.data = float(filtered_value)
            self.publisher_.publish(height_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserToDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()