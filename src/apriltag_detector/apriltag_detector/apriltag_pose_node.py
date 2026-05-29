#!/usr/bin/env python3
"""
Nó ROS2 que detecta AprilTags em imagens de câmera e publica:
  - a pose 6D de cada tag (no frame óptico da câmera);
  - um TF camera_optical_frame -> tag_<id>;
  - o ID lido (no log).

Detecção: cv2.aruco com dicionário AprilTag (ex.: DICT_APRILTAG_36h11).
Pose: cv2.solvePnP com IPPE_SQUARE (otimizado pra marker planar quadrado).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__('apriltag_pose_node')

        self.declare_parameter('tag_size', 0.05)                       
        self.declare_parameter('tag_family', 'DICT_APRILTAG_36h11')    
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')

        self.tag_size = self.get_parameter('tag_size').value
        family_str = self.get_parameter('tag_family').value
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_optical_frame').value

        
        try:
            dict_id = getattr(cv2.aruco, family_str)
        except AttributeError:
            self.get_logger().error(
                f"Família '{family_str}' não existe em cv2.aruco. "
                "Exemplos válidos: DICT_APRILTAG_36h11, DICT_APRILTAG_25h9, DICT_APRILTAG_16h5."
            )
            raise

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        s = self.tag_size / 2.0
        self.object_points = np.array([
            [-s,  s, 0.0],   
            [ s,  s, 0.0],   
            [ s, -s, 0.0],   
            [-s, -s, 0.0],   
        ], dtype=np.float32)

        self.K = None
        self.D = None

        self.bridge = CvBridge()
        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.create_subscription(CameraInfo, info_topic, self.info_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/tag_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Detector iniciado. Família={family_str}  tag_size={self.tag_size} m'
        )


    def info_cb(self, msg: CameraInfo):
        """Captura intrínsecos. K é 3x3 row-major; D pode ter 5 ou 8 coefs."""
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)


    def image_cb(self, msg: Image):
        if self.K is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge falhou: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        for tag_corners, tag_id in zip(corners, ids.flatten()):
            image_points = tag_corners.reshape(4, 2).astype(np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.K,
                self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            rmat, _ = cv2.Rodrigues(rvec)
            quat = R.from_matrix(rmat).as_quat()

            tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])

     
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.camera_frame
            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = tz
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            self.pose_pub.publish(pose)

       
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.camera_frame
            tf.child_frame_id = f'tag_{int(tag_id)}'
            tf.transform.translation.x = tx
            tf.transform.translation.y = ty
            tf.transform.translation.z = tz
            tf.transform.rotation.x = float(quat[0])
            tf.transform.rotation.y = float(quat[1])
            tf.transform.rotation.z = float(quat[2])
            tf.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf)

         #   Log do ID e da posição 
            distance = float(np.linalg.norm(tvec))
            self.get_logger().info(
                f'Tag id={int(tag_id)}  '
                f'pos=({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) m  '
                f'd={distance:.3f} m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
