#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from scipy.ndimage import median_filter
from collections import deque

class OpticalFlowOdometer:
    def __init__(self, intrinsic_matrix, dist_coeffs, scale=0.5, image_size=(1920,1080)):
        # Uses Shi-Tomasi + pyramidal LK optical flow with depth for vertical (Y) translation estimation.
        self.scale      = scale
        self.K          = intrinsic_matrix.copy()
        self.distCoeffs = dist_coeffs

        # Precompute undistort maps for full-res frames
        w, h = image_size
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.distCoeffs, np.eye(3), self.K, (w, h), cv2.CV_32FC1
        )

        # Scaled intrinsics for downsampled images
        self.Ks = self.K.copy()
        self.Ks[0,0] *= scale  # fx
        self.Ks[1,1] *= scale  # fy
        self.Ks[0,2] *= scale  # cx
        self.Ks[1,2] *= scale  # cy

        # Parameters for Shi-Tomasi feature detection
        self.feature_params = dict(
            maxCorners=500,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7
        )

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        self.prev_gray  = None
        self.prev_depth = None
        self.prev_pts   = None
        self.dz_buffer  = deque(maxlen=5)

    def filter_depth(self, img):
        return median_filter(img, size=5)

    def undistort(self, img):
        return cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

    def initialize(self, gray, depth):
        self.prev_gray  = gray
        self.prev_depth = depth
        self.prev_pts   = cv2.goodFeaturesToTrack(gray, **self.feature_params)

    def get_vertical_delta(self, rgb, depth):
        # Undistort full-res
        rgb_u   = self.undistort(rgb)
        depth_u = self.undistort(depth)

        # Gray and downsample
        gray_full = cv2.cvtColor(rgb_u, cv2.COLOR_BGR2GRAY)
        h, w      = gray_full.shape[:2]
        gray_s    = cv2.resize(gray_full, (int(w*self.scale), int(h*self.scale)))
        depth_s   = cv2.resize(depth_u,  (int(w*self.scale), int(h*self.scale)))
        depth_s   = self.filter_depth(depth_s)
        Hs, Ws    = depth_s.shape

        if self.prev_gray is None:
            self.initialize(gray_s, depth_s)
            return None

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray_s, self.prev_pts, None, **self.lk_params
        )
        if next_pts is None:
            self.initialize(gray_s, depth_s)
            return None

        good_prev = self.prev_pts[status.flatten() == 1]
        good_next = next_pts[status.flatten() == 1]

        t_ys = []
        for (x0, y0), (x1, y1) in zip(good_prev.reshape(-1, 2), good_next.reshape(-1, 2)):
            xi0, yi0 = int(x0), int(y0)
            xi1, yi1 = int(x1), int(y1)
            # skip if outside image bounds
            if not (0 <= xi0 < Ws and 0 <= yi0 < Hs and 0 <= xi1 < Ws and 0 <= yi1 < Hs):
                continue
            z0 = depth_s[yi0, xi0]
            z1 = depth_s[yi1, xi1]
            if z0 == 0 or z1 == 0:
                continue
            # back-project to mm
            X0 = (x0 - self.Ks[0,2]) * z0 / self.Ks[0,0]
            Y0 = (y0 - self.Ks[1,2]) * z0 / self.Ks[1,1]
            X1 = (x1 - self.Ks[0,2]) * z1 / self.Ks[0,0]
            Y1 = (y1 - self.Ks[1,2]) * z1 / self.Ks[1,1]
            t_ys.append(Y1 - Y0)

        # Update points
        if len(good_next) >= 10:
            self.prev_pts = good_next.reshape(-1,1,2)
        else:
            self.prev_pts = cv2.goodFeaturesToTrack(gray_s, **self.feature_params)
        self.prev_gray  = gray_s
        self.prev_depth = depth_s

        if len(t_ys) < 6:
            return None

        median_dy = float(np.median(t_ys))
        self.dz_buffer.append(median_dy)
        smooth_dy = float(np.median(self.dz_buffer))

        return smooth_dy / 1000.0

class FlowZTracker(Node):
    def __init__(self, skip_frames=5):
        super().__init__('flow_z_tracker')

        self.declare_parameter('rgb_topic',            '/camera/color/image_raw')
        self.declare_parameter('depth_topic',          '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic',    '/camera/color/camera_info')
        self.declare_parameter('vertical_delta_topic', '/camera_vertical_delta')
        self.declare_parameter('skip_frames',          5)

        self.rgb_topic   = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic  = self.get_parameter('camera_info_topic').value
        self.delta_topic = self.get_parameter('vertical_delta_topic').value
        self.skip_frames = int(self.get_parameter('skip_frames').value)


        self.bridge      = CvBridge()
        self.z_offset    = 0.0
        #self.ground      = 0.0
        self._frame_count = 0

        # note
        # camera params set by info from camera_info topic in bag.
        # Clearly this changes, I was having errors reading from topic, just set it manually
        # This can be improved on if genuinely used/managed
        K = np.array([
            [902.9823, 0.0,      956.5549],
            [0.0,      902.7744, 547.6816],
            [0.0,      0.0,      1.0]
        ], dtype=np.float64)
        dist_coeffs = np.array([
            0.23445, -2.52125, 0.00077, -0.00011,
            1.57097,  0.10949, -2.31927,  1.48007
        ], dtype=np.float64)

        self.odometer = OpticalFlowOdometer(K, dist_coeffs, scale=0.5, image_size=(1920,1080))

        self.caminfo_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.on_camera_info, 10
        )
        self.z_pub = self.create_publisher(Float32, self.delta_topic, 10)

        rgb_sub   = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        ts.registerCallback(self.on_frame)

        self.get_logger().info("FlowZTracker ready with LK optical flow + depth smoothing.")

    def on_camera_info(self, msg: CameraInfo):
        K    = np.array(msg.k, dtype=np.float64).reshape(3,3)
        dist = np.array(msg.d, dtype=np.float64)
        self.odometer.K = K
        self.odometer.distCoeffs = dist
        self.get_logger().info("CameraInfo loaded, will continue with smoothed flow delta.")
        self.destroy_subscription(self.caminfo_sub)

    def on_frame(self, rgb_msg, depth_msg):
        self._frame_count += 1
        if self._frame_count % self.skip_frames != 0:
            return

        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        dz = self.odometer.get_vertical_delta(rgb, depth)
        if dz is None:
            self.get_logger().debug("Skipped frame: insufficient valid points")
            return

        self.z_offset += dz
        #z_world = self.z_offset + self.ground
        z_world = self.z_offset
        self.z_pub.publish(Float32(data=float(z_world)))
        # at this point realized that I can just track an offset var (at a timestep) and a total var (over all time domain)
        # then publish both at the same time...
        # but I already added this support in tree_template, it's fine for now :)
        self.get_logger().info(f"ΔZ: {dz:+.3f} m | Total Z: {self.z_offset:.3f} m")


def main():
    rclpy.init()
    node = FlowZTracker(skip_frames=5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
