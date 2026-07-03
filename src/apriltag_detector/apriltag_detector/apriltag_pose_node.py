#!/usr/bin/env python3

import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as R


# ==============================================================================
# Classe que representa uma AprilTag detectada
# ==============================================================================

class AprilTag:
    def __init__(self, tag_id: int, pose: PoseStamped, cor: str):
        self.id   = tag_id
        self.pose = pose
        self.cor  = cor

    def __repr__(self):
        pos = self.pose.pose.position
        return f"AprilTag(id={self.id}, cor='{self.cor}', x={pos.x:+.3f}, y={pos.y:+.3f}, z={pos.z:+.3f})"


# ==============================================================================
# Utilitários de cor
# ==============================================================================

COLOR_BGR_MAP = {
    'vermelho':     (60,  60,  220),
    'azul':         (220, 80,  40),
    'desconhecida': (160, 160, 160),
}


def classify_color_hsv(bgr):
    pixel = np.uint8([[list(bgr)]])
    hsv   = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])
    if v < 50 or s < 50:
        return 'desconhecida'
    if h < 10 or h > 160:
        return 'vermelho'
    if 85 <= h < 130:
        return 'azul'
    return 'desconhecida'


def sample_tag_color(frame, corners):
    pts    = corners.reshape(4, 2).astype(np.float32)
    center = pts.mean(axis=0)
    h, w   = frame.shape[:2]

    def clip_pts(p):
        p = p.copy()
        p[:, 0] = np.clip(p[:, 0], 0, w - 1)
        p[:, 1] = np.clip(p[:, 1], 0, h - 1)
        return p

    rings = [(1.05, 1.30), (1.30, 1.60), (1.60, 2.00)]
    votes: dict[str, int] = {}

    for inner_f, outer_f in rings:
        outer = clip_pts(center + outer_f * (pts - center))
        inner = clip_pts(center + inner_f * (pts - center))
        mask  = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, outer.astype(np.int32), 255)
        cv2.fillConvexPoly(mask, inner.astype(np.int32), 0)
        pixels = frame[mask == 255]
        if len(pixels) == 0:
            continue
        color = classify_color_hsv(pixels.mean(axis=0))
        if color != 'desconhecida':
            votes[color] = votes.get(color, 0) + 1

    return max(votes, key=lambda c: votes[c]) if votes else 'desconhecida'


def draw_tag_overlay(img, corners, tag_id, color_name):
    pts    = corners.reshape(4, 2).astype(np.int32)
    accent = COLOR_BGR_MAP.get(color_name, (160, 160, 160))
    cv2.polylines(img, [pts], isClosed=True, color=accent, thickness=2, lineType=cv2.LINE_AA)
    label = f'{color_name} {tag_id}'
    font  = cv2.FONT_HERSHEY_SIMPLEX
    fs, th = 0.45, 1
    (tw, th_), _ = cv2.getTextSize(label, font, fs, th)
    x0 = int(pts[:, 0].min())
    y0 = int(pts[:, 1].min())
    cv2.rectangle(img, (x0, y0 - th_ - 6), (x0 + tw + 6, y0), accent, -1)
    cv2.putText(img, label, (x0 + 3, y0 - 4), font, fs, (255, 255, 255), th, cv2.LINE_AA)


# ==============================================================================
# Node principal
# ==============================================================================

class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__('apriltag_pose_node')

        self.declare_parameter('tag_size',          0.038)
        self.declare_parameter('tag_family',        'DICT_APRILTAG_36h11')
        self.declare_parameter('image_topic',       '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_scale',   2.0)
        self.declare_parameter('try_inverted',      True)
        self.declare_parameter('log_interval_sec',  1.0)

        self.tag_size        = float(self.get_parameter('tag_size').value)
        self.detection_scale = float(self.get_parameter('detection_scale').value)
        self.try_inverted    = bool(self.get_parameter('try_inverted').value)
        self.log_interval    = float(self.get_parameter('log_interval_sec').value)

        family      = self.get_parameter('tag_family').value
        image_topic = self.get_parameter('image_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value

        # Comprimentos dos braços (do ScaraControl)
        self.l1 = 0.425
        self.l2 = 0.34

        # Ângulos atuais das juntas (rad), atualizados via /imus_yaw
        self.theta1 = 0.0
        self.theta2 = 0.0

        # Detector
        dict_id         = getattr(cv2.aruco, family)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.params     = cv2.aruco.DetectorParameters()
        self.setup_detector_params()
        self.detector   = cv2.aruco.ArucoDetector(self.dictionary, self.params)

        s = self.tag_size / 2.0
        self.object_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float32)

        self.K = None
        self.D = None

        self.bridge    = CvBridge()
        self._last_log: dict[int, float] = {}

        # Lista de tags detectadas no frame atual
        self.detected_tags: list[AprilTag] = []

        # Publishers
        self.pose_cam_pub   = self.create_publisher(PoseStamped,        '/apriltag/tag_pose_camera', 10)
        self.debug_pub      = self.create_publisher(Image,              '/apriltag/debug_image',     10)
        self.block_pose_pub = self.create_publisher(Float32MultiArray,  '/block_pose',               10)

        self.tf_pub = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Image,             image_topic,  self.image_cb, 10)
        self.create_subscription(CameraInfo,        info_topic,   self.info_cb,  10)
        self.create_subscription(Float32MultiArray, '/imus_yaw',  self.imu_cb,   10)

        self.get_logger().info(
            f'AprilTag iniciado | tag_size={self.tag_size:.4f} m | l1={self.l1} | l2={self.l2}'
        )

    def set_param(self, name, value):
        if hasattr(self.params, name):
            setattr(self.params, name, value)

    def setup_detector_params(self):
        self.set_param('cornerRefinementMethod',        cv2.aruco.CORNER_REFINE_SUBPIX)
        self.set_param('cornerRefinementWinSize',       5)
        self.set_param('cornerRefinementMaxIterations', 80)
        self.set_param('cornerRefinementMinAccuracy',   0.001)
        self.set_param('adaptiveThreshWinSizeMin',      3)
        self.set_param('adaptiveThreshWinSizeMax',      83)
        self.set_param('adaptiveThreshWinSizeStep',     4)
        self.set_param('minMarkerPerimeterRate',        0.005)
        self.set_param('maxMarkerPerimeterRate',        4.0)
        self.set_param('polygonalApproxAccuracyRate',   0.08)
        self.set_param('minCornerDistanceRate',         0.005)
        self.set_param('minMarkerDistanceRate',         0.005)
        self.set_param('minDistanceToBorder',           0)
        self.set_param('errorCorrectionRate',           1.0)
        self.set_param('detectInvertedMarker',          True)
        self.set_param('aprilTagQuadDecimate',          1.0)
        self.set_param('aprilTagQuadSigma',             0.0)
        self.set_param('aprilTagMinClusterPixels',      5)
        self.set_param('aprilTagMaxNmaxima',            20)
        self.set_param('aprilTagMaxLineFitMse',         20.0)
        self.set_param('aprilTagMinWhiteBlackDiff',     3)

    def imu_cb(self, msg):
        # [yaw_base, yaw_arm1, yaw_arm2, yaw_gripper] em radianos
        self.theta1 = msg.data[1]
        self.theta2 = msg.data[2]

    def info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros(5)

    def image_cb(self, msg):
        if self.K is None:
            self.get_logger().warn('Ainda sem CameraInfo.', throttle_duration_sec=2.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge falhou: {e}')
            return

        camera_frame = msg.header.frame_id
        stamp        = msg.header.stamp

        gray            = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_tags(gray)
        corners, ids    = self.filter_duplicates(corners, ids)

        tag_colors: dict[int, str] = {}
        if ids is not None:
            for tag_corners, tag_id in zip(corners, ids.flatten()):
                tag_colors[int(tag_id)] = sample_tag_color(frame, tag_corners)

        self.publish_debug(frame, corners, ids, tag_colors, stamp, camera_frame)

        self.detected_tags = []

        if ids is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        for tag_corners, tag_id in zip(corners, ids.flatten()):
            tid        = int(tag_id)
            color_name = tag_colors[tid]

            last = self._last_log.get(tid, 0.0)
            if now - last >= self.log_interval:
                self._last_log[tid] = now
                self.get_logger().info(f'[TAG] ID={tid} | cor={color_name}')

            self.compute_pose(tag_corners, tid, color_name, stamp, camera_frame)

    def detect_tags(self, gray):
        images = [(gray, False)]
        if self.try_inverted:
            images.append((cv2.bitwise_not(gray), True))

        for image, _ in images:
            detect_image = image
            if self.detection_scale != 1.0:
                detect_image = cv2.resize(
                    image, None,
                    fx=self.detection_scale, fy=self.detection_scale,
                    interpolation=cv2.INTER_CUBIC
                )
            corners, ids, rejected = self.detector.detectMarkers(detect_image)
            if self.detection_scale != 1.0 and corners is not None:
                corners = [c.astype(np.float32) / self.detection_scale for c in corners]
            if ids is not None:
                return corners, ids, rejected

        return None, None, None

    def filter_duplicates(self, corners, ids):
        if ids is None:
            return corners, ids

        best = {}
        for c, tag_id in zip(corners, ids.flatten()):
            pts  = c.reshape(4, 2)
            side = np.linalg.norm(pts[0] - pts[1])
            tid  = int(tag_id)
            if tid not in best or side > best[tid][0]:
                best[tid] = (side, c)

        new_corners, new_ids = [], []
        for tid, (_, c) in best.items():
            new_corners.append(c)
            new_ids.append([tid])

        return new_corners, np.array(new_ids, dtype=np.int32)

    def compute_pose(self, tag_corners, tag_id, color_name, stamp, camera_frame):
        image_points = tag_corners.reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            self.object_points, image_points, self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                self.object_points, image_points, self.K, self.D,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            self.get_logger().warn(f'solvePnP falhou para tag {tag_id}')
            return

        rmat, _ = cv2.Rodrigues(rvec)
        quat    = R.from_matrix(rmat).as_quat()

        # Pose no frame da câmera
        pose_cam = PoseStamped()
        pose_cam.header.stamp    = stamp
        pose_cam.header.frame_id = camera_frame
        pose_cam.pose.position.x = float(tvec[0][0])
        pose_cam.pose.position.y = float(tvec[1][0])
        pose_cam.pose.position.z = float(tvec[2][0])
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])
        self.pose_cam_pub.publish(pose_cam)
        self.publish_tf(tag_id, stamp, tvec, quat, camera_frame)

        # Adiciona à lista de tags do frame atual
        tag = AprilTag(tag_id=tag_id, pose=pose_cam, cor=color_name)
        self.detected_tags.append(tag)

        # ------------------------------------------------------------------
        # Cálculo da posição do bloco no frame da base do manipulador
        #
        # x' = tvec[0]: deslocamento lateral da tag no frame da câmera
        # y' = tvec[1]: deslocamento frontal da tag no frame da câmera
        #
        # θ' = atan2(y', x')
        # L3 = sqrt(x'² + y'²)
        # θ3 = θ' - π/2
        #
        # x = L1·cos(θ1) + L2·cos(θ1+θ2) + L3·cos(θ1+θ2+θ3)
        # y = L1·sin(θ1) + L2·sin(θ1+θ2) + L3·sin(θ1+θ2+θ3)
        # ------------------------------------------------------------------
        x_prime = float(tvec[0][0])
        y_prime = float(tvec[1][0])

        theta_prime = math.atan2(y_prime, x_prime)
        L3          = math.sqrt(x_prime**2 + y_prime**2)
        theta3      = theta_prime - math.pi / 2.0

        x_block = (self.l1 * math.cos(self.theta1)
                 + self.l2 * math.cos(self.theta1 + self.theta2)
                 + L3      * math.cos(self.theta1 + self.theta2 + theta3))

        y_block = (self.l1 * math.sin(self.theta1)
                 + self.l2 * math.sin(self.theta1 + self.theta2)
                 + L3      * math.sin(self.theta1 + self.theta2 + theta3))

        block_msg      = Float32MultiArray()
        block_msg.data = [float(x_block), float(y_block)]
        self.block_pose_pub.publish(block_msg)

        self.get_logger().info(
            f'[BLOCK] Tag {tag_id} | x\'={x_prime:+.4f} y\'={y_prime:+.4f} | '
            f'L3={L3:.4f} | x={x_block:+.4f} y={y_block:+.4f}',
            throttle_duration_sec=self.log_interval
        )

    def publish_tf(self, tag_id, stamp, tvec, quat, camera_frame):
        tf                         = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = camera_frame
        tf.child_frame_id          = f'tag_{tag_id}'
        tf.transform.translation.x = float(tvec[0][0])
        tf.transform.translation.y = float(tvec[1][0])
        tf.transform.translation.z = float(tvec[2][0])
        tf.transform.rotation.x    = float(quat[0])
        tf.transform.rotation.y    = float(quat[1])
        tf.transform.rotation.z    = float(quat[2])
        tf.transform.rotation.w    = float(quat[3])
        self.tf_pub.sendTransform(tf)

    def publish_debug(self, frame, corners, ids, tag_colors, stamp, camera_frame):
        debug = frame.copy()
        if ids is not None and len(ids) > 0:
            for tag_corners, tag_id in zip(corners, ids.flatten()):
                draw_tag_overlay(debug, tag_corners, int(tag_id), tag_colors.get(int(tag_id), 'desconhecida'))
        try:
            msg                 = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            msg.header.stamp    = stamp
            msg.header.frame_id = camera_frame
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Erro no debug: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()