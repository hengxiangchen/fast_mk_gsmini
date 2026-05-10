#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from loguru import logger

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField, CompressedImage
from geometry_msgs.msg import WrenchStamped

from utils.tactile_marker_utils import marker_normalization
from utils import find_marker
from utils.gs_utility import GelsightUtility
from utils.gs_driver import Gsmini


class GelsightCameraPublisherROS1(object):
    """
    ROS1 版 GelSight Camera Publisher (全功能优化版)
    功能:
    1. 发布原始图像 (压缩)
    2. 发布可视化图像 (带蓝色箭头，压缩) 【新增】
    3. 发布 Marker 位移点云
    4. 发布接触力 Wrench
    5. 高效的 While 循环结构，无 deepcopy
    """

    def __init__(self,
                 camera_name='gelsight_camera',
                 dimension=2,
                 fps=30,
                 debug=True,
                 video_path="./data/tactile_video/video_001.mp4",
                 recorded=False):
        
        # 配置参数
        self.camera_name = camera_name
        self.fps = fps
        self.width = 640
        self.height = 480
        self.debug = debug
        self.dimension = dimension
        self.recorded = recorded
        
        # 图像容器
        self.img = None
        self.marker_img = None # 用于存储带箭头的可视化图像

        # Marker 追踪状态
        self.initial_markers = None
        self.marker_motion = None
        self.initial_markers_3d = None
        self.vertical_scale = 0.05

        # FPS 统计
        self.prev_time = time.time()
        self.frame_count = 0
        self.last_print_time = time.time()

        # --------------------- ROS Publishers --------------------- #
        
        # 1. 原始颜色图像发布 (压缩)
        self.color_pub = rospy.Publisher(
            f'/{camera_name}/color/image_raw/compressed', CompressedImage, queue_size=1
        )

        # 2. 【新增】可视化图像发布 (带箭头，压缩)
        # 话题名加了 /vis 前缀以示区别
        self.vis_pub = rospy.Publisher(
            f'/{camera_name}/vis/image_raw/compressed', CompressedImage, queue_size=1
        )
        
        # 3. Marker 位移点云发布
        self.marker_pub = rospy.Publisher(
            f'/{camera_name}/marker_offset/information', PointCloud2, queue_size=1
        )
        # 4. 接触力发布
        self.force_pub = rospy.Publisher(
            f'/{camera_name}/contact_force', WrenchStamped, queue_size=1
        )

        # 相机初始化
        self.video_path = video_path
        self.cap = None
        self.gsmini = None

        if self.recorded:
            if not os.path.exists(self.video_path):
                raise FileNotFoundError(f"Video path {self.video_path} does not exist!")
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video {self.video_path}")
            logger.info(f"{self.camera_name} playing video {self.video_path}")
        else:
            self.gsmini = Gsmini()
            logger.info(f"{self.camera_name} started using Gsmini driver.")

        # GelSight 算法工具初始化
        self.GelsightHandler = GelsightUtility(RESCALE=1)
        self.m = find_marker.Matching(
            N_=self.GelsightHandler.N,
            M_=self.GelsightHandler.M,
            fps_=self.GelsightHandler.fps,
            x0_=self.GelsightHandler.x0,
            y0_=self.GelsightHandler.y0,
            dx_=self.GelsightHandler.dx,
            dy_=self.GelsightHandler.dy,
        )

    # --------------------- 图像处理 --------------------- #

    def get_rgb_frame(self):
        """获取并预处理图像"""
        frame = None
        timestamp = rospy.Time.now()

        if self.recorded:
            if self.cap is None:
                raise RuntimeError("Video capture is not initialized.")
            ret, frame = self.cap.read()
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret: return None, timestamp
        else:
            if not self.gsmini.connect():
                raise RuntimeError("Failed to connect to Gsmini camera")
            frame, timestamp = self.gsmini._get_image()

        self.img = frame
        
        # 调整大小并初始化
        resized_frame = cv2.resize(frame, (self.width, self.height))
        processed_frame = self.GelsightHandler.img_initiation(resized_frame)
        
        return processed_frame, timestamp

    def get_marker_image(self, img):
        """Marker 检测、追踪并生成可视化图像"""
        mask = self.GelsightHandler.find_marker(img)
        markers_detected = self.GelsightHandler.marker_center(mask)

        self.initial_markers, self.marker_motion = self.track_marker(markers_detected, self.dimension)

        # 【关键修改】始终生成带有箭头的可视化图像
        # 这样无论是否开启本地调试窗口，都可以发布这个图像
        img_show = img.copy()
        # showScale=5 可以调整箭头显示的长度比例
        self.marker_img = self.display_motion(img_show, self.initial_markers, self.marker_motion,
                                              showScale=5, dimension=self.dimension)
        
        return self.initial_markers, self.marker_motion

    def track_marker(self, marker_center, dimension):
        """执行 Matching 算法"""
        self.m.init(marker_center)
        self.m.run()

        flow = self.m.get_flow()
        Ox, Oy, Cx, Cy, _ = flow
        M, N = len(Ox), len(Ox[0])

        if self.initial_markers_3d is None:
            self.initial_markers_3d = self.GelsightHandler.ComputesurroundingArea(Ox, Oy)

        initial_marker = np.zeros((M * N, 3), dtype=np.float32)
        marker_motion = np.zeros((M * N, 2), dtype=np.float32)

        if dimension == 3:
            current_marker_3d = self.GelsightHandler.ComputesurroundingArea(Cx, Cy)

        k = 0
        for i in range(M):
            for j in range(N):
                if self.dimension == 2:
                    initial_marker[k] = [Ox[i][j], Oy[i][j], 0.0]
                elif self.dimension == 3:
                    dz = (current_marker_3d[i][j] - self.initial_markers_3d[i][j]) * self.vertical_scale
                    initial_marker[k] = [Ox[i][j], Oy[i][j], max(dz, 0.0)]
                
                marker_motion[k] = [Cx[i][j] - Ox[i][j], Cy[i][j] - Oy[i][j]]
                k += 1

        return initial_marker, marker_motion

    @staticmethod
    def display_motion(img_show, initial_markers, marker_motions, showScale=1, dimension=2):
        """在图像上绘制蓝色箭头"""
        markerCenter = np.around(initial_markers[:, 0:2]).astype(np.int16)
        for i in range(initial_markers.shape[0]):
            if marker_motions[i, 0] != 0 or marker_motions[i, 1] != 0:
                end_point = (
                    int(initial_markers[i, 0] + marker_motions[i, 0] * showScale),
                    int(initial_markers[i, 1] + marker_motions[i, 1] * showScale),
                )
                end_point = (
                    np.clip(end_point[0], 0, img_show.shape[1] - 1),
                    np.clip(end_point[1], 0, img_show.shape[0] - 1),
                )
                # 绘制箭头，颜色为蓝色 (B,G,R) = (255, 0, 0)
                cv2.arrowedLine(img_show, (markerCenter[i, 0], markerCenter[i, 1]), end_point, (255, 0, 0), 2)
        return img_show

    # --------------------- ROS 发布函数 --------------------- #

    def publish_marker_offset(self, marker_loc, marker_offset, camera_timestamp):
        # 快速序列化
        cur_marker = marker_loc[:, :2]
        marker_information = np.hstack((cur_marker, marker_offset)).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = camera_timestamp
        msg.header.frame_id = f'camera_marker_offset_{self.camera_name}'
        msg.is_bigendian = False
        msg.point_step = 16
        msg.is_dense = True
        msg.fields = [
            PointField(name='marker_location_x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='marker_location_y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='marker_offset_x', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='marker_offset_y', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = marker_information.tobytes()
        self.marker_pub.publish(msg)

    def publish_color_image(self, color_image, camera_timestamp):
        """发布原始颜色图像 (JPEG 压缩)"""
        success, encoded_image = cv2.imencode('.jpg', color_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        if success:
            msg = CompressedImage()
            msg.header.stamp = camera_timestamp
            msg.header.frame_id = f"camera_color_frame_{self.camera_name}"
            msg.format = "jpeg"
            msg.data = encoded_image.tobytes()
            self.color_pub.publish(msg)
        else:
            logger.error("Failed to encode color image!")

    def publish_vis_image(self, vis_image, camera_timestamp):
        """
        【新增】发布带有箭头的可视化图像 (JPEG 压缩)
        """
        if vis_image is None:
            return

        # 同样使用 JPEG 压缩以节省带宽
        success, encoded_image = cv2.imencode('.jpg', vis_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        if success:
            msg = CompressedImage()
            msg.header.stamp = camera_timestamp
            # 使用相同的 frame_id，保证时间同步
            msg.header.frame_id = f"camera_color_frame_{self.camera_name}"
            msg.format = "jpeg"
            msg.data = encoded_image.tobytes()
            self.vis_pub.publish(msg)
        else:
            logger.error("Failed to encode visualization image!")

    def compute_contact_forces(self, initial_markers, marker_motion, k_n, k_t, dimension=3):
        initial_markers = np.asarray(initial_markers)
        marker_motion = np.asarray(marker_motion)

        Ft_per_marker = k_t * marker_motion.astype(np.float32)

        if dimension == 3 and initial_markers.shape[1] >= 3:
            dz = initial_markers[:, 2].astype(np.float32)
            Fn_per_marker = k_n * dz
        else:
            Fn_per_marker = np.zeros(marker_motion.shape[0], dtype=np.float32)

        F_total = np.array([
            np.sum(Ft_per_marker[:, 0]), 
            np.sum(Ft_per_marker[:, 1]), 
            np.sum(Fn_per_marker)
        ], dtype=np.float32)

        return Fn_per_marker, Ft_per_marker, F_total

    def run(self):
        """
        主循环: 高效同步模式
        """
        logger.info(f"Starting loop at {self.fps} Hz (Dual Image Mode)")
        rate = rospy.Rate(self.fps)

        while not rospy.is_shutdown():
            # 1. 获取图像
            color_frame, initial_time = self.get_rgb_frame()
            if color_frame is None:
                rate.sleep()
                continue

            # 2. Marker 算法 (这一步会生成 self.marker_img)
            initial_markers, marker_motion = self.get_marker_image(color_frame)

            # 3. 力计算
            k_n, k_t = 1.0, 1.0
            _, _, F_total = self.compute_contact_forces(initial_markers, marker_motion, k_n, k_t, dimension=3)

            # 发布力
            force_msg = WrenchStamped()
            force_msg.header.stamp = initial_time
            force_msg.header.frame_id = f'gelsight_force_{self.camera_name}'
            force_msg.wrench.force.x = float(F_total[0])
            force_msg.wrench.force.y = float(F_total[1])
            force_msg.wrench.force.z = float(F_total[2])
            self.force_pub.publish(force_msg)

            # 4. 归一化 Marker 数据
            initial_markers_norm, marker_motion_norm = marker_normalization(
                initial_markers.copy(),
                marker_motion.copy(),
                self.dimension,
                width=self.width,
                height=self.height,
            )

            # 5. 发布所有数据
            self.publish_marker_offset(initial_markers_norm, marker_motion_norm, initial_time)
            self.publish_color_image(color_frame, initial_time)    # 发布原始图
            self.publish_vis_image(self.marker_img, initial_time)  # 【新增】发布可视化图

            # 6. 调试显示与 FPS
            self.frame_count += 1
            curr_time = time.time()
            if curr_time - self.prev_time >= 1.0:
                fps = self.frame_count / (curr_time - self.prev_time)
                logger.debug(f"FPS: {fps:.2f}")
                self.prev_time = curr_time
                self.frame_count = 0

            # # 本地调试窗口 (可选，cv2.imshow 会轻微阻塞)
            if self.debug and self.marker_img is not None:
                cv2.imshow('marker_debug_local', self.marker_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()


if __name__ == "__main__":
    # 线程优化配置
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "4")
    os.environ.setdefault("MKL_NUM_THREADS", "4")
    os.environ.setdefault("OMP_NUM_THREADS", "4")
    cv2.setNumThreads(4)

    rospy.init_node("gelsight_camera_publisher_ros1", anonymous=True)

    node = GelsightCameraPublisherROS1(
        camera_name="gelsight_camera",
        debug=True,       # 调试模式 (开启本地窗口)
        recorded=False,   # 实时模式
        dimension=2,
        fps=30
    )

    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node.gsmini:
            node.gsmini.release()
        if node.cap:
            node.cap.release()
        cv2.destroyAllWindows()