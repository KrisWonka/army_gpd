#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import threading
import time
import yaml

import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from hiwonder_interfaces.msg import MultiRawIdPosDur, RawIdPosDur
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Log
from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import tf2_ros
    HAS_TF2 = True
except Exception:
    HAS_TF2 = False

# Reuse camera/depth and settings module from old UI.
from jetarm_ui_node import UiConfig, RosBridge, SettingsDialog

try:
    from gpd_ros.msg import CloudIndexed, CloudSources, GraspConfigList
    from gpd_ros.srv import detect_grasps
    HAS_GPD_SRV = True
except Exception:
    HAS_GPD_SRV = False


def rpy_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_to_rot(qx, qy, qz, qw):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def norm_frame(name):
    s = str(name or "").strip()
    while s.startswith("/"):
        s = s[1:]
    return s


def is_optical_frame(name):
    return norm_frame(name).endswith("_optical_frame")


# ---------------------------------------------------------------------------
# i18n: all translatable UI strings   key -> (english, chinese)
# ---------------------------------------------------------------------------
TR = {
    "win_title":        ("TF Calibration + Grasp Control",       "TF 校准 + 抓取控制"),
    "power":            ("Power",                                 "总开关"),
    "settings":         ("Settings",                              "参数设置"),
    "lang_toggle":      ("中文",                                   "English"),
    "control_panel":    ("Control Panel",                         "控制面板"),
    "grab":             ("Grab",                                  "抓取"),
    "store":            ("Store",                                 "存放"),
    "release":          ("Release",                               "释放"),
    "init_pos":         ("Init Position",                         "回到初始位置"),
    "auto_store":       ("Auto Store",                            "自动存放"),
    "status_standby":   ("Status: Standby",                       "状态: 待机"),
    "status_on":        ("Status: Running",                       "状态: 已开启"),
    "status_off":       ("Status: Off",                           "状态: 已关闭"),
    "status_grabbing":  ("Status: Grabbing",                      "状态: 抓取中"),
    "status_grab_ok":   ("Status: Grab OK",                       "状态: 抓取完成"),
    "status_grab_fail": ("Status: Grab Failed",                   "状态: 抓取失败"),
    "status_wait_sel":  ("Status: Awaiting selection",            "状态: 等待你选择候选"),
    "pre_grab_delay":   ("Pre-grab Delay",                        "抓取前等待"),
    "wait_duration":    ("Wait Duration",                         "等待时长"),
    "save_delay":       ("Save Delay",                            "保存等待"),
    "speed_ctrl":       ("Speed Control",                         "运动调速"),
    "slow":             ("Slow",                                  "慢"),
    "med":              ("Med",                                   "中"),
    "fast":             ("Fast",                                  "快"),
    "save_speed":       ("Save Speed",                            "保存速度"),
    "camera_feed":      ("Camera Feed",                           "相机画面"),
    "camera":           ("Camera",                                "相机画面"),
    "depth":            ("Depth",                                 "深度画面"),
    "tf_calib":         ("TF Calibration",                        "TF调节"),
    "gpd_tuning":       ("GPD Grasp Tuning",                      "GPD 抓取调节"),
    "attack_angle":     ("Attack Angle (0~90)",                   "攻击角 (0~90)"),
    "approach_dist":    ("Approach Dist.",                         "向前距离"),
    "edge_margin":      ("Edge Margin",                           "避边距离"),
    "edge_penalty":     ("Edge Penalty",                          "避边权重"),
    "cluster_radius":   ("Cluster Radius",                        "聚类半径"),
    "proj_offset_u":    ("Proj. Offset U(right+)",                "投影偏移 U(右+)"),
    "proj_offset_v":    ("Proj. Offset V(down+)",                 "投影偏移 V(下+)"),
    "save":             ("Save",                                  "保存"),
    "reload":           ("Reload",                                "重新加载"),
    "log":              ("Log",                                   "日志"),
    "gpd_scan_title":   ("GPD Single Scan Progress",              "GPD 单次扫描进度"),
    "stage_standby":    ("Stage: Standby",                        "阶段: 待机"),
    "stage_preparing":  ("Stage: Preparing",                      "阶段: 准备开始"),
    "stage_fmt":        ("Stage: {}",                             "阶段: {}"),
    "picker_title":     ("GPD Candidate Selection",               "GPD 候选抓取选择"),
    "picker_waiting":   ("Waiting for camera & GPD candidates…",  "等待相机与GPD候选..."),
    "clear_sel":        ("Clear Selection",                       "清除选择"),
    "grab_selected":    ("Grab Selected",                         "按当前选择抓取"),
    "sel_auto":         ("Selected: Auto",                        "已选候选: 自动"),
    "sel_fmt":          ("Selected: #{}",                         "已选候选: #{}"),
    "sel_auto_off":     ("Selected: Auto (picker off)",           "已选候选: 自动（点选关闭）"),
    "picker_off":       ("Picker disabled (enable_grasp_picker:=true)",
                         "候选点选已关闭（启动参数 enable_grasp_picker:=true 可开启）"),
    "picker_wait_new":  ("Waiting for new candidates…",           "等待本轮新候选..."),
    "picker_no_cand":   ("No candidates this round",              "本轮无候选夹取点"),
    "picker_enabled":   ("Enabled",                               "开启"),
    "picker_disabled":  ("Disabled",                              "关闭"),
    "proj_ok":          ("Projection OK {} → {}",                 "投影成功 {} -> {}"),
    "proj_fail_px":     ("Proj. fail: {} → {} TF ok, no pixels",  "投影失败: {} -> {} 有TF但无有效像素点"),
    "proj_fail_tf":     ("Proj. fail: {} → {} no TF",             "投影失败: {} -> {} 无可用TF"),
    "proj_fallback":    ("{}, TOP list only",                     "{}，仅显示TOP候选列表"),
    "running":          ("Running",                               "运行中"),
    "done":             ("Done",                                  "完成"),
    "exception":        ("Exception",                             "异常"),
    "cancel":           ("Cancel",                                "取消"),
}


def _cluster_grasps(grasps_scored, radius):
    """Cluster spatially close grasps and average each cluster.

    *grasps_scored*: list of (score, original_idx, grasp_msg)
    *radius*: clustering radius in metres

    Returns list of (avg_score, best_idx, avg_x, avg_y, avg_z) per cluster,
    sorted by avg_score descending.  *best_idx* is the original index of the
    highest-scoring member (used to communicate with gpd_grasp_node).
    """
    clusters = []
    for score, idx, g in grasps_scored:
        if not hasattr(g, "position"):
            continue
        px, py, pz = float(g.position.x), float(g.position.y), float(g.position.z)
        merged = False
        for c in clusters:
            dx = px - c["sx"] / c["n"]
            dy = py - c["sy"] / c["n"]
            dz = pz - c["sz"] / c["n"]
            if dx * dx + dy * dy + dz * dz <= radius * radius:
                c["sx"] += px
                c["sy"] += py
                c["sz"] += pz
                c["stotal"] += score
                c["n"] += 1
                if score > c["best_score"]:
                    c["best_score"] = score
                    c["best_idx"] = idx
                merged = True
                break
        if not merged:
            clusters.append({
                "sx": px, "sy": py, "sz": pz,
                "stotal": score, "n": 1,
                "best_score": score, "best_idx": idx,
            })
    result = []
    for c in clusters:
        n = c["n"]
        result.append((
            c["stotal"] / n,
            c["best_idx"],
            c["sx"] / n,
            c["sy"] / n,
            c["sz"] / n,
        ))
    result.sort(key=lambda x: x[0], reverse=True)
    return result


class ClickableLabel(QtWidgets.QLabel):
    clicked = QtCore.pyqtSignal(int, int)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit(int(event.x()), int(event.y()))
        super().mousePressEvent(event)


class GpdProgressDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, lang="en"):
        super().__init__(parent)
        self._lang = lang
        self.setModal(False)
        self.resize(420, 220)
        lay = QtWidgets.QVBoxLayout(self)
        self.stage_label = QtWidgets.QLabel()
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.detail_view = QtWidgets.QPlainTextEdit()
        self.detail_view.setReadOnly(True)
        lay.addWidget(self.stage_label)
        lay.addWidget(self.progress)
        lay.addWidget(self.detail_view, 1)
        self.retranslate(lang)

    def retranslate(self, lang):
        self._lang = lang
        idx = 0 if lang == "en" else 1
        self.setWindowTitle(TR["gpd_scan_title"][idx])
        self.stage_label.setText(TR["stage_standby"][idx])

    def reset_for_new_run(self):
        idx = 0 if self._lang == "en" else 1
        self.stage_label.setText(TR["stage_preparing"][idx])
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.detail_view.clear()

    def update_progress(self, pct, stage, detail):
        pct = max(0, min(100, int(pct)))
        idx = 0 if self._lang == "en" else 1
        self.stage_label.setText(TR["stage_fmt"][idx].format(stage))
        self.progress.setRange(0, 100)
        self.progress.setValue(pct)
        if detail:
            self.detail_view.appendPlainText(detail)
            sb = self.detail_view.verticalScrollBar()
            sb.setValue(sb.maximum())


class CandidatePickerDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, lang="en"):
        super().__init__(parent)
        self._lang = lang
        self.setModal(False)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        self.resize(860, 720)
        lay = QtWidgets.QVBoxLayout(self)
        self.view_label = ClickableLabel()
        self.view_label.setAlignment(QtCore.Qt.AlignCenter)
        self.view_label.setMinimumSize(820, 480)
        self.view_label.setStyleSheet("background-color:#202020;color:#cccccc;")
        lay.addWidget(self.view_label, 3)
        self.candidates_list = QtWidgets.QListWidget()
        self.candidates_list.setMinimumHeight(180)
        lay.addWidget(self.candidates_list, 2)
        row = QtWidgets.QHBoxLayout()
        self.clear_btn = QtWidgets.QPushButton()
        self.confirm_btn = QtWidgets.QPushButton()
        self.selected_label = QtWidgets.QLabel()
        row.addWidget(self.clear_btn)
        row.addWidget(self.confirm_btn)
        row.addWidget(self.selected_label, 1)
        lay.addLayout(row)
        self.retranslate(lang)

    def retranslate(self, lang):
        self._lang = lang
        idx = 0 if lang == "en" else 1
        self.setWindowTitle(TR["picker_title"][idx])
        self.view_label.setText(TR["picker_waiting"][idx])
        self.clear_btn.setText(TR["clear_sel"][idx])
        self.confirm_btn.setText(TR["grab_selected"][idx])
        self.selected_label.setText(TR["sel_auto"][idx])


class TfCalibrationControlUi(QtWidgets.QWidget):
    log_signal = QtCore.pyqtSignal(str)
    grab_result_signal = QtCore.pyqtSignal(bool, str)
    gpd_progress_signal = QtCore.pyqtSignal(int, str, str)
    gpd_log_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rospy.init_node("tf_calibration_control_ui", anonymous=True)

        default_tf_cfg = os.path.expanduser("~/.jetarm_ui/tf_calibration.yaml")
        legacy_tf_cfg = "/home/hiwonder/jetarm/src/jetarm_ui/config/tf_calibration.yaml"
        self.config_path = rospy.get_param(
            "~config_path",
            default_tf_cfg,
        )
        self.legacy_tf_config_path = rospy.get_param("~legacy_config_path", legacy_tf_cfg)
        self.ui_config_path = rospy.get_param(
            "~ui_config_path",
            "/home/hiwonder/jetarm/src/jetarm_ui/config/ui_config.yaml",
        )
        self.parent_frame = rospy.get_param("~parent_frame", "link5")
        self.child_frame = rospy.get_param("~child_frame", "rgbd_cam_link")
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.grasp_topic = rospy.get_param("~grasp_topic", "/detect_grasps/clustered_grasps")
        self.grasp_trigger_service = rospy.get_param("~grasp_trigger_service", "/gpd_grasp/trigger")
        self.grasp_clear_cache_service = rospy.get_param("~grasp_clear_cache_service", "/gpd_grasp/clear_cache")
        self.use_single_shot_scan = rospy.get_param("~use_single_shot_scan", True)
        self.single_shot_cloud_topic = rospy.get_param("~single_shot_cloud_topic", "/rgbd_cam/depth/points_base")
        self.single_shot_gpd_service = rospy.get_param("~single_shot_gpd_service", "/gpd_once_server/detect_grasps")
        self.single_shot_stride = int(rospy.get_param("~single_shot_stride", 40))
        self.grab_delay_before_scan_s = float(rospy.get_param("~grab_delay_before_scan_s", 12.0))
        self.wait_timeout_s = float(rospy.get_param("~wait_timeout_s", 60.0))
        self.auto_store_delay_s = float(rospy.get_param("~auto_store_delay_s", 1.2))
        self.auto_return_init_delay_s = float(rospy.get_param("~auto_return_init_delay_s", 3.0))
        self.gripper_servo_id = int(rospy.get_param("~gripper_servo_id", 10))
        # JetArm default usually treats smaller value as more open.
        self.gripper_open_position = int(rospy.get_param("~gripper_open_position", 380))
        self.gripper_duration_ms = int(rospy.get_param("~gripper_duration_ms", 500))
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/rgbd_cam/color/camera_info")
        self.grasp_points_frame = rospy.get_param("~grasp_points_frame", "base_link")
        self.camera_frame_fallbacks = [
            "rgbd_cam_color_optical_frame",
            "rgbd_cam_depth_optical_frame",
            "rgbd_cam_link",
        ]
        self.enable_grasp_picker = bool(rospy.get_param("~enable_grasp_picker", True))
        self.selection_wait_timeout_s = float(rospy.get_param("~selection_wait_timeout_s", 25.0))
        self.candidate_top_k = int(rospy.get_param("~candidate_top_k", 6))

        self.values = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": -1.57,
            "yaw": 0.0,
        }
        self.grasp_offset = {"x": -0.03, "y": 0.0, "z": 0.0}
        self.motion_speed_percent = 100
        self.attack_angle_deg = 20.0
        self.approach_distance_m = 0.05
        self.edge_margin_m = 0.03
        self.edge_penalty_weight = 3.0
        self.picker_offset_u = 0
        self.picker_offset_v = 0
        self.cluster_radius_m = 0.03
        self._lang = "en"
        self._grab_busy = False
        self._collect_gpd_logs = False
        self._camera_model = None
        self._latest_qimg = None
        self._latest_grasp_msg = None
        self._candidate_points_2d = []
        self._selected_candidate_index = -1
        self._candidate_draw_meta = None
        self._last_candidate_render_t = 0.0
        self._last_projection_pair = None
        self._projected_candidates_cache = []
        self._projected_candidates_cache_t = 0.0
        self._selection_confirmed = False
        self._selection_waiting = False
        self._selection_event = threading.Event()
        self._tf_edges = {}
        self._tf_static_edges = {}
        self._candidate_list_signature = None
        self._candidate_list_selected = None
        self._projection_status = "等待候选..."
        self._last_projection_status = ""
        self._cached_clustered = []
        self._cached_cluster_grasp_id = None

        self.ui_config = UiConfig(self.ui_config_path)
        self.ros_bridge = RosBridge(self.ui_config)
        self.settings_dialog = SettingsDialog(self.ui_config, self.ros_bridge, self)
        self.settings_dialog.setModal(False)
        self.gpd_progress_dialog = GpdProgressDialog(self, lang=self._lang)
        self.candidate_dialog = CandidatePickerDialog(self, lang=self._lang)
        self.ui_state = QtCore.QSettings("jetarm_ui", "tf_calibration_control_ui")
        try:
            self.settings_dialog.pose_tabs.setTabText(2, "存放姿态")
            self.settings_dialog.pose_tabs.setTabText(3, "第一层存放姿态")
        except Exception:
            pass

        self.servos_pub = rospy.Publisher("/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.gpd_grasps_pub = rospy.Publisher(self.grasp_topic, GraspConfigList, queue_size=1) if HAS_GPD_SRV else None
        self.rosout_sub = rospy.Subscriber("/rosout_agg", Log, self._on_rosout_log, queue_size=200)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self._on_tf_msg, queue_size=100)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self._on_tf_static_msg, queue_size=20)
        self.camera_info_sub = None
        if self.enable_grasp_picker:
            self.camera_info_sub = rospy.Subscriber(
                self.camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1
            )
        self.tf_buffer = None
        self.tf_listener = None
        if self.enable_grasp_picker and HAS_TF2:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        elif self.enable_grasp_picker:
            self.log_signal.emit("警告: tf2_ros 不可用，候选投影点击功能降级")
        if self.enable_grasp_picker and HAS_GPD_SRV:
            self.grasp_sub = rospy.Subscriber(self.grasp_topic, GraspConfigList, self._on_grasp_candidates, queue_size=1)
        else:
            self.grasp_sub = None

        self._build_ui()
        self.log_signal.emit(
            "候选点选: {} | 等待确认超时: {}s".format(
                "开启" if self.enable_grasp_picker else "关闭",
                int(self.selection_wait_timeout_s),
            )
        )
        try:
            rospy.set_param("/gpd_grasp/selected_grasp_index", -1)
        except Exception:
            pass
        self._load_tf_config()
        self._apply_to_widgets()
        self._set_controls_enabled(False)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._publish_tf)
        self.timer.start(max(10, int(1000.0 / self.publish_hz)))

        self.log_signal.connect(self._append_log)
        self.grab_result_signal.connect(self._on_grab_result)
        self.gpd_progress_signal.connect(self._on_gpd_progress)
        self.gpd_log_signal.connect(self._on_gpd_log_line)
        self.ros_bridge.log_signal.connect(self._append_log)
        self.ros_bridge.image_signal.connect(self._update_image)
        self.ros_bridge.depth_image_signal.connect(self._update_depth_image)
        self.candidate_dialog.view_label.clicked.connect(self._on_candidate_view_clicked)
        self.candidate_dialog.clear_btn.clicked.connect(self._clear_selected_candidate)
        self.candidate_dialog.confirm_btn.clicked.connect(self._confirm_candidate_selection)
        self.candidate_dialog.candidates_list.itemClicked.connect(self._on_candidate_list_clicked)
        self._restore_candidate_dialog_geometry()

    def _tr(self, key):
        idx = 0 if self._lang == "en" else 1
        return TR.get(key, ("?", "?"))[idx]

    def _build_ui(self):
        self.setWindowTitle(self._tr("win_title"))
        self.resize(1540, 900)
        main = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        self.power_switch = QtWidgets.QCheckBox(self._tr("power"))
        self.power_switch.setChecked(False)
        self.settings_btn = QtWidgets.QPushButton(self._tr("settings"))
        self.lang_btn = QtWidgets.QPushButton(self._tr("lang_toggle"))
        self.lang_btn.setFixedWidth(68)
        top.addWidget(self.power_switch)
        top.addWidget(self.settings_btn)
        top.addStretch()
        top.addWidget(self.lang_btn)
        main.addLayout(top)

        body = QtWidgets.QHBoxLayout()
        main.addLayout(body, 1)

        left_box = QtWidgets.QGroupBox(self._tr("control_panel"))
        left_v = QtWidgets.QVBoxLayout(left_box)
        self.grab_btn = QtWidgets.QPushButton(self._tr("grab"))
        self.store_btn = QtWidgets.QPushButton(self._tr("store"))
        self.release_btn = QtWidgets.QPushButton(self._tr("release"))
        self.init_btn = QtWidgets.QPushButton(self._tr("init_pos"))
        self.auto_store_cb = QtWidgets.QCheckBox(self._tr("auto_store"))
        self.status_label = QtWidgets.QLabel(self._tr("status_standby"))
        left_v.addWidget(self.grab_btn)
        left_v.addWidget(self.store_btn)
        left_v.addWidget(self.release_btn)
        left_v.addWidget(self.init_btn)
        left_v.addWidget(self.auto_store_cb)
        left_v.addWidget(self.status_label)

        delay_group = QtWidgets.QGroupBox(self._tr("pre_grab_delay"))
        delay_layout = QtWidgets.QHBoxLayout(delay_group)
        self.grab_delay_spin = QtWidgets.QDoubleSpinBox()
        self.grab_delay_spin.setDecimals(1)
        self.grab_delay_spin.setRange(0.0, 60.0)
        self.grab_delay_spin.setSingleStep(0.5)
        self.grab_delay_spin.setSuffix(" s")
        self.grab_delay_save_btn = QtWidgets.QPushButton(self._tr("save_delay"))
        self.delay_duration_label = QtWidgets.QLabel(self._tr("wait_duration"))
        delay_layout.addWidget(self.delay_duration_label)
        delay_layout.addWidget(self.grab_delay_spin, 1)
        delay_layout.addWidget(self.grab_delay_save_btn)
        left_v.addWidget(delay_group)

        speed_group = QtWidgets.QGroupBox(self._tr("speed_ctrl"))
        speed_v = QtWidgets.QVBoxLayout(speed_group)
        speed_row = QtWidgets.QHBoxLayout()
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(30, 120)
        self.speed_spin = QtWidgets.QSpinBox()
        self.speed_spin.setRange(30, 120)
        self.speed_spin.setSuffix("%")
        speed_row.addWidget(self.speed_slider, 1)
        speed_row.addWidget(self.speed_spin, 0)
        speed_v.addLayout(speed_row)
        preset_row = QtWidgets.QHBoxLayout()
        self.speed_slow_btn = QtWidgets.QPushButton(self._tr("slow"))
        self.speed_mid_btn = QtWidgets.QPushButton(self._tr("med"))
        self.speed_fast_btn = QtWidgets.QPushButton(self._tr("fast"))
        preset_row.addWidget(self.speed_slow_btn)
        preset_row.addWidget(self.speed_mid_btn)
        preset_row.addWidget(self.speed_fast_btn)
        speed_v.addLayout(preset_row)
        self.speed_save_btn = QtWidgets.QPushButton(self._tr("save_speed"))
        speed_v.addWidget(self.speed_save_btn)
        left_v.addWidget(speed_group)
        left_v.addStretch()
        body.addWidget(left_box, 0)

        center_box = QtWidgets.QGroupBox(self._tr("camera_feed"))
        center_v = QtWidgets.QVBoxLayout(center_box)
        self.video_label = QtWidgets.QLabel(self._tr("camera"))
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(860, 300)
        self.video_label.setStyleSheet("background-color:#222;color:#ddd;")
        self.depth_label = QtWidgets.QLabel(self._tr("depth"))
        self.depth_label.setAlignment(QtCore.Qt.AlignCenter)
        self.depth_label.setMinimumSize(860, 300)
        self.depth_label.setStyleSheet("background-color:#222;color:#ddd;")
        center_v.addWidget(self.video_label, 1)
        center_v.addWidget(self.depth_label, 1)
        body.addWidget(center_box, 2)

        right_box = QtWidgets.QGroupBox(self._tr("tf_calib"))
        right_v = QtWidgets.QVBoxLayout(right_box)

        frame_group = QtWidgets.QGroupBox("Frame")
        frame_layout = QtWidgets.QFormLayout(frame_group)
        self.parent_edit = QtWidgets.QLineEdit(self.parent_frame)
        self.child_edit = QtWidgets.QLineEdit(self.child_frame)
        frame_layout.addRow("Parent Frame", self.parent_edit)
        frame_layout.addRow("Child Frame", self.child_edit)
        right_v.addWidget(frame_group)

        self.controls = {}
        params = [
            ("x", -0.50, 0.50, 0.001),
            ("y", -0.50, 0.50, 0.001),
            ("z", -0.50, 0.50, 0.001),
            ("roll", -3.14, 3.14, 0.01),
            ("pitch", -3.14, 3.14, 0.01),
            ("yaw", -3.14, 3.14, 0.01),
        ]
        tf_grid = QtWidgets.QGridLayout()
        tf_grid.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        tf_grid.addWidget(QtWidgets.QLabel("Slider"), 0, 1)
        tf_grid.addWidget(QtWidgets.QLabel("Value"), 0, 2)
        for row, (name, vmin, vmax, step) in enumerate(params, start=1):
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(3 if step < 0.01 else 2)
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            slider.setRange(0, int(round((vmax - vmin) / step)))

            def on_slider(val, key=name, mn=vmin, st=step, sp=spin):
                real = mn + val * st
                sp.blockSignals(True)
                sp.setValue(real)
                sp.blockSignals(False)
                self.values[key] = float(real)
                self._update_preview()

            def on_spin(val, key=name, mn=vmin, st=step, sl=slider):
                idx = int(round((val - mn) / st))
                sl.blockSignals(True)
                sl.setValue(idx)
                sl.blockSignals(False)
                self.values[key] = float(val)
                self._update_preview()

            slider.valueChanged.connect(on_slider)
            spin.valueChanged.connect(on_spin)
            tf_grid.addWidget(label, row, 0)
            tf_grid.addWidget(slider, row, 1)
            tf_grid.addWidget(spin, row, 2)
            self.controls[name] = (slider, spin, vmin, step)
        right_v.addLayout(tf_grid)

        grasp_group = QtWidgets.QGroupBox("Grasp Offset (base_link)")
        grasp_grid = QtWidgets.QGridLayout(grasp_group)
        grasp_grid.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        grasp_grid.addWidget(QtWidgets.QLabel("Slider"), 0, 1)
        grasp_grid.addWidget(QtWidgets.QLabel("Value"), 0, 2)
        self.grasp_controls = {}
        for row, (name, vmin, vmax, step) in enumerate(
            [("x", -0.10, 0.10, 0.001), ("y", -0.10, 0.10, 0.001), ("z", -0.10, 0.10, 0.001)],
            start=1,
        ):
            label = QtWidgets.QLabel(name)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            slider.setRange(0, int(round((vmax - vmin) / step)))

            def on_slider(val, key=name, mn=vmin, st=step, sp=spin):
                real = mn + val * st
                sp.blockSignals(True)
                sp.setValue(real)
                sp.blockSignals(False)
                self.grasp_offset[key] = float(real)
                self._update_preview()

            def on_spin(val, key=name, mn=vmin, st=step, sl=slider):
                idx = int(round((val - mn) / st))
                sl.blockSignals(True)
                sl.setValue(idx)
                sl.blockSignals(False)
                self.grasp_offset[key] = float(val)
                self._update_preview()

            slider.valueChanged.connect(on_slider)
            spin.valueChanged.connect(on_spin)
            grasp_grid.addWidget(label, row, 0)
            grasp_grid.addWidget(slider, row, 1)
            grasp_grid.addWidget(spin, row, 2)
            self.grasp_controls[name] = (slider, spin, vmin, step)
        right_v.addWidget(grasp_group)

        gpd_tuning_group = QtWidgets.QGroupBox(self._tr("gpd_tuning"))
        gpd_tuning_form = QtWidgets.QFormLayout(gpd_tuning_group)
        self.attack_angle_spin = QtWidgets.QDoubleSpinBox()
        self.attack_angle_spin.setDecimals(1)
        self.attack_angle_spin.setRange(0.0, 90.0)
        self.attack_angle_spin.setSingleStep(1.0)
        self.attack_angle_spin.setSuffix(" deg")
        self.approach_distance_spin = QtWidgets.QDoubleSpinBox()
        self.approach_distance_spin.setDecimals(3)
        self.approach_distance_spin.setRange(0.005, 0.300)
        self.approach_distance_spin.setSingleStep(0.005)
        self.approach_distance_spin.setSuffix(" m")
        self.edge_margin_spin = QtWidgets.QDoubleSpinBox()
        self.edge_margin_spin.setDecimals(3)
        self.edge_margin_spin.setRange(0.0, 0.100)
        self.edge_margin_spin.setSingleStep(0.005)
        self.edge_margin_spin.setSuffix(" m")
        self.edge_penalty_spin = QtWidgets.QDoubleSpinBox()
        self.edge_penalty_spin.setDecimals(2)
        self.edge_penalty_spin.setRange(0.0, 20.0)
        self.edge_penalty_spin.setSingleStep(0.5)
        self.cluster_radius_spin = QtWidgets.QDoubleSpinBox()
        self.cluster_radius_spin.setDecimals(3)
        self.cluster_radius_spin.setRange(0.005, 0.200)
        self.cluster_radius_spin.setSingleStep(0.005)
        self.cluster_radius_spin.setSuffix(" m")
        self.attack_angle_label = QtWidgets.QLabel(self._tr("attack_angle"))
        self.approach_dist_label = QtWidgets.QLabel(self._tr("approach_dist"))
        self.edge_margin_label = QtWidgets.QLabel(self._tr("edge_margin"))
        self.edge_penalty_label = QtWidgets.QLabel(self._tr("edge_penalty"))
        self.cluster_radius_label = QtWidgets.QLabel(self._tr("cluster_radius"))
        gpd_tuning_form.addRow(self.attack_angle_label, self.attack_angle_spin)
        gpd_tuning_form.addRow(self.approach_dist_label, self.approach_distance_spin)
        gpd_tuning_form.addRow(self.edge_margin_label, self.edge_margin_spin)
        gpd_tuning_form.addRow(self.edge_penalty_label, self.edge_penalty_spin)
        gpd_tuning_form.addRow(self.cluster_radius_label, self.cluster_radius_spin)
        self.picker_offset_u_spin = QtWidgets.QSpinBox()
        self.picker_offset_u_spin.setRange(-200, 200)
        self.picker_offset_u_spin.setSingleStep(5)
        self.picker_offset_u_spin.setSuffix(" px")
        self.picker_offset_v_spin = QtWidgets.QSpinBox()
        self.picker_offset_v_spin.setRange(-200, 200)
        self.picker_offset_v_spin.setSingleStep(5)
        self.picker_offset_v_spin.setSuffix(" px")
        self.proj_u_label = QtWidgets.QLabel(self._tr("proj_offset_u"))
        self.proj_v_label = QtWidgets.QLabel(self._tr("proj_offset_v"))
        gpd_tuning_form.addRow(self.proj_u_label, self.picker_offset_u_spin)
        gpd_tuning_form.addRow(self.proj_v_label, self.picker_offset_v_spin)
        right_v.addWidget(gpd_tuning_group)

        self.preview = QtWidgets.QPlainTextEdit()
        self.preview.setReadOnly(True)
        right_v.addWidget(self.preview, 1)

        btn_row = QtWidgets.QHBoxLayout()
        self.save_btn = QtWidgets.QPushButton(self._tr("save"))
        self.reload_btn = QtWidgets.QPushButton(self._tr("reload"))
        btn_row.addWidget(self.save_btn)
        btn_row.addWidget(self.reload_btn)
        btn_row.addStretch()
        right_v.addLayout(btn_row)
        body.addWidget(right_box, 1)

        log_box = QtWidgets.QGroupBox(self._tr("log"))
        log_v = QtWidgets.QVBoxLayout(log_box)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        log_v.addWidget(self.log_view)
        main.addWidget(log_box, 1)

        self.power_switch.stateChanged.connect(self._on_power_toggled)
        self.settings_btn.clicked.connect(self._on_settings)
        self.grab_btn.clicked.connect(self._on_grab_clicked)
        self.store_btn.clicked.connect(self._on_store_clicked)
        self.release_btn.clicked.connect(self._on_release_clicked)
        self.init_btn.clicked.connect(lambda: self._move_pose_from_ui_config("init"))
        self.save_btn.clicked.connect(self._save_all_config)
        self.reload_btn.clicked.connect(self._reload_all_config)
        self.parent_edit.textChanged.connect(self._update_preview)
        self.child_edit.textChanged.connect(self._update_preview)
        self.speed_slider.valueChanged.connect(self._on_speed_slider_changed)
        self.speed_spin.valueChanged.connect(self._on_speed_spin_changed)
        self.speed_slow_btn.clicked.connect(lambda: self._apply_speed_preset(45))
        self.speed_mid_btn.clicked.connect(lambda: self._apply_speed_preset(65))
        self.speed_fast_btn.clicked.connect(lambda: self._apply_speed_preset(85))
        self.speed_save_btn.clicked.connect(self._save_motion_config)
        self.grab_delay_spin.valueChanged.connect(self._on_grab_delay_changed)
        self.grab_delay_save_btn.clicked.connect(self._save_all_config)
        self.attack_angle_spin.valueChanged.connect(self._on_attack_angle_changed)
        self.approach_distance_spin.valueChanged.connect(self._on_approach_distance_changed)
        self.edge_margin_spin.valueChanged.connect(self._on_edge_margin_changed)
        self.edge_penalty_spin.valueChanged.connect(self._on_edge_penalty_changed)
        self.picker_offset_u_spin.valueChanged.connect(self._on_picker_offset_u_changed)
        self.picker_offset_v_spin.valueChanged.connect(self._on_picker_offset_v_changed)
        self.cluster_radius_spin.valueChanged.connect(self._on_cluster_radius_changed)
        self.lang_btn.clicked.connect(self._toggle_language)
        if not self.enable_grasp_picker:
            self.candidate_dialog.view_label.setText(self._tr("picker_off"))
            self.candidate_dialog.clear_btn.setEnabled(False)
            self.candidate_dialog.selected_label.setText(self._tr("sel_auto_off"))

    def _append_log(self, msg):
        self.log_view.appendPlainText(msg)
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _toggle_language(self):
        self._lang = "zh" if self._lang == "en" else "en"
        self._retranslate_ui()
        try:
            self.ui_state.setValue("language", self._lang)
        except Exception:
            pass

    def _retranslate_ui(self):
        t = self._tr
        self.setWindowTitle(t("win_title"))
        self.power_switch.setText(t("power"))
        self.settings_btn.setText(t("settings"))
        self.lang_btn.setText(t("lang_toggle"))
        self.grab_btn.setText(t("grab"))
        self.store_btn.setText(t("store"))
        self.release_btn.setText(t("release"))
        self.init_btn.setText(t("init_pos"))
        self.auto_store_cb.setText(t("auto_store"))
        self.grab_delay_save_btn.setText(t("save_delay"))
        self.delay_duration_label.setText(t("wait_duration"))
        self.speed_slow_btn.setText(t("slow"))
        self.speed_mid_btn.setText(t("med"))
        self.speed_fast_btn.setText(t("fast"))
        self.speed_save_btn.setText(t("save_speed"))
        self.save_btn.setText(t("save"))
        self.reload_btn.setText(t("reload"))
        self.attack_angle_label.setText(t("attack_angle"))
        self.approach_dist_label.setText(t("approach_dist"))
        self.edge_margin_label.setText(t("edge_margin"))
        self.edge_penalty_label.setText(t("edge_penalty"))
        self.cluster_radius_label.setText(t("cluster_radius"))
        self.proj_u_label.setText(t("proj_offset_u"))
        self.proj_v_label.setText(t("proj_offset_v"))
        for gb in self.findChildren(QtWidgets.QGroupBox):
            name = gb.objectName()
            if not name:
                continue
        # Re-set group box titles via parent layout lookup
        try:
            self.grab_btn.parentWidget().setTitle(t("control_panel"))
            self.grab_delay_spin.parentWidget().parentWidget().setTitle(t("pre_grab_delay"))
            self.speed_slider.parentWidget().parentWidget().setTitle(t("speed_ctrl"))
            self.video_label.parentWidget().setTitle(t("camera_feed"))
            self.parent_edit.parentWidget().parentWidget().setTitle(t("tf_calib"))
            self.attack_angle_spin.parentWidget().setTitle(t("gpd_tuning"))
            self.log_view.parentWidget().setTitle(t("log"))
        except Exception:
            pass
        self.gpd_progress_dialog.retranslate(self._lang)
        self.candidate_dialog.retranslate(self._lang)

    def _on_cluster_radius_changed(self, value):
        self.cluster_radius_m = float(value)
        self._projected_candidates_cache = []
        self._cached_cluster_grasp_id = None

    def _set_controls_enabled(self, enabled):
        self.grab_btn.setEnabled(enabled and not self._grab_busy)
        self.store_btn.setEnabled(enabled and not self._grab_busy)
        self.release_btn.setEnabled(enabled and not self._grab_busy)
        self.init_btn.setEnabled(enabled and not self._grab_busy)
        self.settings_btn.setEnabled(enabled)

    def _publish_servo_pose(self, duration_ms, joints):
        msg = MultiRawIdPosDur()
        msg.id_pos_dur_list = []
        for pair in joints:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            item = RawIdPosDur()
            item.id = int(pair[0])
            item.position = int(pair[1])
            item.duration = int(duration_ms)
            msg.id_pos_dur_list.append(item)
        if msg.id_pos_dur_list:
            self.servos_pub.publish(msg)

    def _scaled_duration_ms(self, cfg, duration_ms):
        motion = cfg.get("motion", {}) if isinstance(cfg, dict) else {}
        speed_percent = float(motion.get("speed_percent", self.motion_speed_percent))
        self.motion_speed_percent = int(max(30, min(120, round(speed_percent))))
        speed_scale = max(0.1, min(self.motion_speed_percent / 100.0, 2.0))
        return int(max(50, float(duration_ms) / speed_scale))

    def _move_pose_from_ui_config(self, pose_key):
        cfg = self._read_ui_config()
        pose = ((cfg.get("poses") or {}).get(pose_key) or {})
        joints = pose.get("joints", [])
        duration_ms = int(pose.get("duration_ms", 800))
        real_duration_ms = self._scaled_duration_ms(cfg, duration_ms)
        if not joints:
            self.log_signal.emit("姿态 {} 为空".format(pose_key))
            return
        self._publish_servo_pose(real_duration_ms, joints)
        self.log_signal.emit(
            "执行姿态 {} (基准 {} ms, 速度 {}%, 实际 {} ms)".format(
                pose_key, duration_ms, self.motion_speed_percent, real_duration_ms
            )
        )

    def _start_required_services(self):
        self.ros_bridge.start()
        self.ros_bridge.call_empty("/color_detection/enter")
        self.ros_bridge.call_empty("/color_detection/start")

    def _stop_required_services(self):
        self.ros_bridge.call_empty("/color_detection/stop")
        self.ros_bridge.call_empty("/color_detection/exit")
        self.ros_bridge.stop()

    def _on_power_toggled(self, state):
        on = state == QtCore.Qt.Checked
        if on:
            self._set_controls_enabled(True)
            self.status_label.setText(self._tr("status_on"))
            self.log_signal.emit(self._tr("status_on"))
            self._start_required_services()
            self._move_pose_from_ui_config("init")
            self._show_candidate_dialog()
        else:
            self._set_controls_enabled(False)
            self.status_label.setText(self._tr("status_off"))
            self.log_signal.emit(self._tr("status_off"))
            self._stop_required_services()
            if self.candidate_dialog.isVisible():
                self.candidate_dialog.hide()

    def _show_candidate_dialog(self):
        if not self.ui_state.contains("candidate_dialog/geometry"):
            geo = self.geometry()
            self.candidate_dialog.move(geo.x() + geo.width() + 15, geo.y() + 10)
        self.candidate_dialog.show()
        self.candidate_dialog.raise_()
        self.candidate_dialog.activateWindow()

    def _reset_picker_for_new_round(self):
        self._latest_grasp_msg = None
        self._candidate_points_2d = []
        self._candidate_draw_meta = None
        self._projected_candidates_cache = []
        self._projected_candidates_cache_t = 0.0
        self._candidate_list_signature = None
        self._candidate_list_selected = None
        self._cached_clustered = []
        self._cached_cluster_grasp_id = None
        self._selected_candidate_index = -1
        self._selection_confirmed = False
        self._selection_waiting = False
        self._selection_event.clear()
        self._projection_status = self._tr("picker_wait_new")
        self._tf_diag_done = False
        self._proj_sample_done = False
        self.candidate_dialog.candidates_list.blockSignals(True)
        self.candidate_dialog.candidates_list.clear()
        self.candidate_dialog.candidates_list.blockSignals(False)
        self.candidate_dialog.selected_label.setText(self._tr("sel_auto"))
        if self._latest_qimg is not None:
            pix0 = QtGui.QPixmap.fromImage(self._latest_qimg)
            self.candidate_dialog.view_label.setPixmap(
                pix0.scaled(
                    self.candidate_dialog.view_label.size(),
                    QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.SmoothTransformation,
                )
            )
        else:
            self.candidate_dialog.view_label.setText(self._tr("picker_wait_new"))

    def _restore_candidate_dialog_geometry(self):
        try:
            if self.ui_state.contains("candidate_dialog/geometry"):
                geom = self.ui_state.value("candidate_dialog/geometry")
                if geom:
                    self.candidate_dialog.restoreGeometry(geom)
        except Exception:
            pass

    def _save_candidate_dialog_geometry(self):
        try:
            self.ui_state.setValue("candidate_dialog/geometry", self.candidate_dialog.saveGeometry())
        except Exception:
            pass

    def _on_settings(self):
        if self.settings_dialog.isVisible():
            self.settings_dialog.hide()
            return
        geo = self.geometry()
        size = self.settings_dialog.sizeHint()
        self.settings_dialog.resize(size.width(), size.height())
        self.settings_dialog.move(geo.x() + geo.width() + 10, geo.y())
        self.settings_dialog.show()

    def _on_grab_clicked(self):
        if self._grab_busy:
            return
        if self.enable_grasp_picker:
            self._reset_picker_for_new_round()
            try:
                rospy.set_param("/gpd_grasp/selected_grasp_index", -1)
            except Exception:
                pass
        self._grab_busy = True
        self._set_controls_enabled(True)
        self.status_label.setText(self._tr("status_grabbing"))
        if self.use_single_shot_scan:
            self._collect_gpd_logs = True
            self.gpd_progress_dialog.reset_for_new_run()
            geo = self.geometry()
            self.gpd_progress_dialog.move(geo.x() + geo.width() + 15, geo.y() + 30)
            self.gpd_progress_dialog.show()
            self.gpd_progress_signal.emit(5, "回初始姿态", "开始抓取流程")
        self._move_pose_from_ui_config("init")
        th = threading.Thread(target=self._grab_worker)
        th.daemon = True
        th.start()

    def _grab_worker(self):
        try:
            cfg = self._read_ui_config()
            init_pose = ((cfg.get("poses") or {}).get("init") or {})
            init_duration_ms = int(init_pose.get("duration_ms", 800))
            init_real_duration_ms = self._scaled_duration_ms(cfg, init_duration_ms)
            rospy.sleep(max(0.0, init_real_duration_ms / 1000.0 + 0.2))
            if self.use_single_shot_scan:
                if not HAS_GPD_SRV:
                    self.gpd_progress_signal.emit(100, "失败", "gpd_ros service messages unavailable")
                    self.grab_result_signal.emit(False, "gpd_ros service messages unavailable")
                    return
                if self.grab_delay_before_scan_s > 0:
                    self.gpd_progress_signal.emit(
                        15,
                        "稳定等待",
                        "抓取前等待 {:.1f}s".format(self.grab_delay_before_scan_s),
                    )
                    rospy.sleep(self.grab_delay_before_scan_s)
                self.gpd_progress_signal.emit(
                    20,
                    "等待点云",
                    "等待 {} 上的点云（最多 {}s）".format(self.single_shot_cloud_topic, int(self.wait_timeout_s)),
                )
                self.log_signal.emit(
                    "[waiter] waiting up to {}s for cloud {} ...".format(
                        int(self.wait_timeout_s), self.single_shot_cloud_topic
                    )
                )
                discard_n = 3
                cloud_msg = None
                try:
                    for i in range(discard_n + 1):
                        cloud_msg = rospy.wait_for_message(
                            self.single_shot_cloud_topic, PointCloud2, timeout=self.wait_timeout_s
                        )
                        if i < discard_n:
                            self.gpd_progress_signal.emit(
                                20 + i * 3, "刷新点云", "丢弃旧帧 {}/{}".format(i + 1, discard_n)
                            )
                except Exception:
                    self.gpd_progress_signal.emit(100, "超时", "未收到点云")
                    self.grab_result_signal.emit(False, "waiter超时({}s)，未收到点云".format(int(self.wait_timeout_s)))
                    return

                self.gpd_progress_signal.emit(45, "调用检测服务", self.single_shot_gpd_service)
                self.log_signal.emit("[once] call {} ...".format(self.single_shot_gpd_service))
                try:
                    rospy.wait_for_service(self.grasp_clear_cache_service, timeout=1.5)
                    clear_proxy = rospy.ServiceProxy(self.grasp_clear_cache_service, Trigger)
                    clear_proxy()
                    self.gpd_progress_signal.emit(35, "清空旧缓存", self.grasp_clear_cache_service)
                except Exception:
                    # Keep compatibility if clear service is not available.
                    pass
                rospy.wait_for_service(self.single_shot_gpd_service, timeout=3.0)
                srv = rospy.ServiceProxy(self.single_shot_gpd_service, detect_grasps)
                req = detect_grasps._request_class()
                npts = max(1, int(cloud_msg.width * cloud_msg.height))

                cloud_sources = CloudSources()
                cloud_sources.cloud = cloud_msg
                # detect_grasps_server expects one camera_source entry per point.
                # Using all zeros means all points come from camera index 0.
                cloud_sources.camera_source = [Int64(0) for _ in range(npts)]
                cloud_sources.view_points = [Point(0.0, 0.0, 0.0)]

                cloud_indexed = CloudIndexed()
                cloud_indexed.cloud_sources = cloud_sources
                stride = max(1, self.single_shot_stride)
                cloud_indexed.indices = [Int64(i) for i in range(0, npts, stride)]
                req.cloud_indexed = cloud_indexed

                try:
                    res_once = srv(req)
                except rospy.ServiceException:
                    # gpd_ros/detect_grasps_server returns false when no grasps are found,
                    # and rospy surfaces it as a service error. Treat it as a normal miss.
                    self.gpd_progress_signal.emit(100, "完成", "单次扫描无可用抓取点")
                    self.grab_result_signal.emit(False, "单次扫描无可用抓取点")
                    return
                grasp_count = len(res_once.grasp_configs.grasps)
                if grasp_count <= 0:
                    self.gpd_progress_signal.emit(100, "完成", "单次扫描无可用抓取点")
                    self.grab_result_signal.emit(False, "单次扫描无可用抓取点")
                    return
                self.gpd_progress_signal.emit(70, "检测完成", "本次得到 {} 个抓取候选".format(grasp_count))
                self.log_signal.emit("[once] got {} grasps, publish -> {}".format(grasp_count, self.grasp_topic))
                self.gpd_grasps_pub.publish(res_once.grasp_configs)
                self.gpd_progress_signal.emit(80, "发布结果", "已发布到 {}".format(self.grasp_topic))
                rospy.sleep(0.15)
                if self.enable_grasp_picker:
                    self._selection_waiting = True
                    self._selection_confirmed = False
                    self.log_signal.emit(
                        "等待你确认候选（超时 {}s）".format(int(self.selection_wait_timeout_s))
                    )
                    self.gpd_progress_signal.emit(
                        85,
                        "等待选择",
                        "请在候选窗口点击并确认（超时 {}s）".format(int(self.selection_wait_timeout_s)),
                    )
                    self.status_label.setText(self._tr("status_wait_sel"))
                    self._selection_event.wait(timeout=max(0.1, float(self.selection_wait_timeout_s)))
                    self._selection_waiting = False
                    if not (self._selection_confirmed and self._selected_candidate_index >= 0):
                        self.gpd_progress_signal.emit(100, "取消", "未确认候选，已取消本次抓取")
                        self.grab_result_signal.emit(False, "未确认候选，已取消")
                        return
            else:
                self.log_signal.emit("[waiter] waiting up to {}s for {} ...".format(int(self.wait_timeout_s), self.grasp_topic))
                try:
                    rospy.wait_for_message(self.grasp_topic, rospy.AnyMsg, timeout=self.wait_timeout_s)
                except Exception:
                    self.grab_result_signal.emit(False, "waiter超时({}s)，未收到GPD结果".format(int(self.wait_timeout_s)))
                    return

            self.gpd_progress_signal.emit(90, "触发抓取", self.grasp_trigger_service)
            self.log_signal.emit("[waiter] trigger -> {}".format(self.grasp_trigger_service))
            rospy.wait_for_service(self.grasp_trigger_service, timeout=2.0)
            proxy = rospy.ServiceProxy(self.grasp_trigger_service, Trigger)
            res = proxy()
            self.gpd_progress_signal.emit(100, "完成", "trigger success={}".format(bool(res.success)))
            self.grab_result_signal.emit(bool(res.success), str(res.message))
        except Exception as exc:
            self.gpd_progress_signal.emit(100, "异常", str(exc))
            self.grab_result_signal.emit(False, "grab failed: {}".format(str(exc)))

    def _on_grab_result(self, ok, msg):
        self._grab_busy = False
        self._collect_gpd_logs = False
        self._set_controls_enabled(self.power_switch.isChecked())
        if ok:
            self.status_label.setText(self._tr("status_grab_ok"))
            self.log_signal.emit("[waiter] trigger success: {}".format(msg))
            if self.auto_store_cb.isChecked():
                delay_ms = int(max(0.0, self.auto_store_delay_s) * 1000.0)
                self.log_signal.emit("自动存放已开启，{} ms 后执行存放".format(delay_ms))
                QtCore.QTimer.singleShot(delay_ms, self._on_store_clicked)
        else:
            self.status_label.setText(self._tr("status_grab_fail"))
            self.log_signal.emit("[waiter] trigger failed: {}".format(msg))

    def _on_gpd_progress(self, pct, stage, detail):
        if not self.use_single_shot_scan:
            return
        if not self.gpd_progress_dialog.isVisible():
            self.gpd_progress_dialog.show()
        self.gpd_progress_dialog.update_progress(pct, stage, detail)

    def _on_gpd_log_line(self, text):
        if not self.use_single_shot_scan:
            return
        if not self.gpd_progress_dialog.isVisible():
            self.gpd_progress_dialog.show()
        self.gpd_progress_dialog.update_progress(self.gpd_progress_dialog.progress.value(), "运行中", text)

    def _on_rosout_log(self, msg):
        if not self._collect_gpd_logs or not self.use_single_shot_scan:
            return
        if "gpd_once_server" not in (msg.name or ""):
            return
        if msg.level >= Log.ERROR:
            level = "ERROR"
        elif msg.level >= Log.WARN:
            level = "WARN"
        else:
            level = "INFO"
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        self.gpd_log_signal.emit("[{:.3f}] {} {}: {}".format(t, level, msg.name, msg.msg))

    def _on_camera_info(self, msg):
        try:
            fid = norm_frame(msg.header.frame_id)
            if not fid:
                fid = self.camera_frame_fallbacks[0]
            self._camera_model = {
                "frame_id": fid,
                "fx": float(msg.K[0]),
                "fy": float(msg.K[4]),
                "cx": float(msg.K[2]),
                "cy": float(msg.K[5]),
                "width": int(msg.width),
                "height": int(msg.height),
            }
            # Some drivers publish abnormal K; keep projection usable.
            if self._camera_model["cx"] <= 0 or self._camera_model["cx"] >= self._camera_model["width"]:
                self._camera_model["cx"] = 0.5 * max(1, self._camera_model["width"])
            if self._camera_model["cy"] <= 0 or self._camera_model["cy"] >= self._camera_model["height"]:
                self._camera_model["cy"] = 0.5 * max(1, self._camera_model["height"])
        except Exception:
            pass

    def _on_grasp_candidates(self, msg):
        if not self.enable_grasp_picker:
            return
        self._latest_grasp_msg = msg
        self._candidate_list_signature = None
        self._cached_cluster_grasp_id = None
        self._render_candidate_overlay()

    def _on_tf_msg(self, msg):
        now = time.time()
        for t in msg.transforms:
            child = norm_frame(t.child_frame_id)
            parent = norm_frame(t.header.frame_id)
            if not child or not parent:
                continue
            self._tf_edges[child] = (
                parent,
                float(t.transform.translation.x),
                float(t.transform.translation.y),
                float(t.transform.translation.z),
                float(t.transform.rotation.x),
                float(t.transform.rotation.y),
                float(t.transform.rotation.z),
                float(t.transform.rotation.w),
                now,
            )

    def _on_tf_static_msg(self, msg):
        for t in msg.transforms:
            child = norm_frame(t.child_frame_id)
            parent = norm_frame(t.header.frame_id)
            if not child or not parent:
                continue
            self._tf_static_edges[child] = (
                parent,
                float(t.transform.translation.x),
                float(t.transform.translation.y),
                float(t.transform.translation.z),
                float(t.transform.rotation.x),
                float(t.transform.rotation.y),
                float(t.transform.rotation.z),
                float(t.transform.rotation.w),
                0.0,
            )

    def _quat_mul(self, a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    def _quat_conj(self, q):
        qx, qy, qz, qw = q
        return (-qx, -qy, -qz, qw)

    def _rotate_by_quat(self, q, v):
        vx, vy, vz = v
        vq = (vx, vy, vz, 0.0)
        qq = self._quat_mul(self._quat_mul(q, vq), self._quat_conj(q))
        return (qq[0], qq[1], qq[2])

    def _compose_tf(self, tf_a, tf_b):
        ta, qa = (tf_a[0], tf_a[1], tf_a[2]), (tf_a[3], tf_a[4], tf_a[5], tf_a[6])
        tb, qb = (tf_b[0], tf_b[1], tf_b[2]), (tf_b[3], tf_b[4], tf_b[5], tf_b[6])
        tbr = self._rotate_by_quat(qa, tb)
        out_t = (ta[0] + tbr[0], ta[1] + tbr[1], ta[2] + tbr[2])
        out_q = self._quat_mul(qa, qb)
        return (out_t[0], out_t[1], out_t[2], out_q[0], out_q[1], out_q[2], out_q[3])

    def _invert_tf(self, tf):
        t = (tf[0], tf[1], tf[2])
        q = (tf[3], tf[4], tf[5], tf[6])
        qi = self._quat_conj(q)
        tr = self._rotate_by_quat(qi, (-t[0], -t[1], -t[2]))
        return (tr[0], tr[1], tr[2], qi[0], qi[1], qi[2], qi[3])

    def _lookup_tf_simple(self, target_frame, source_frame):
        target_frame = norm_frame(target_frame)
        source_frame = norm_frame(source_frame)
        if source_frame == target_frame:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        edges = {}
        edges.update(self._tf_static_edges)
        edges.update(self._tf_edges)
        if not edges:
            return None
        adj = {}
        for child, rec in edges.items():
            parent = rec[0]
            tf_pc = (rec[1], rec[2], rec[3], rec[4], rec[5], rec[6], rec[7])
            tf_cp = self._invert_tf(tf_pc)
            adj.setdefault(parent, []).append((child, tf_pc))
            adj.setdefault(child, []).append((parent, tf_cp))
        queue = [(source_frame, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))]
        visited = set([source_frame])
        while queue:
            node, tf_src_to_node = queue.pop(0)
            for nxt, tf_node_to_nxt in adj.get(node, []):
                if nxt in visited:
                    continue
                tf_src_to_nxt = self._compose_tf(tf_src_to_node, tf_node_to_nxt)
                if nxt == target_frame:
                    return tf_src_to_nxt
                visited.add(nxt)
                queue.append((nxt, tf_src_to_nxt))
        return None

    def _project_point_to_image(self, x, y, z, tfm):
        if not self._camera_model:
            return None
        cam = self._camera_model
        try:
            tx, ty, tz, qx, qy, qz, qw = tfm
            r = quat_to_rot(qx, qy, qz, qw)
            cx = r[0][0] * x + r[0][1] * y + r[0][2] * z + tx
            cy = r[1][0] * x + r[1][1] * y + r[1][2] * z + ty
            cz = r[2][0] * x + r[2][1] * y + r[2][2] * z + tz
            if cz <= 1e-6:
                return None
            u = cam["fx"] * (cx / cz) + cam["cx"] + self.picker_offset_u
            v = cam["fy"] * (cy / cz) + cam["cy"] + self.picker_offset_v
            if 0 <= u < cam["width"] and 0 <= v < cam["height"]:
                return int(round(u)), int(round(v))
        except Exception:
            return None
        return None

    def _draw_gripper_marker(self, painter, u, v, color, selected=False):
        pen = QtGui.QPen(color)
        pen.setWidth(3 if selected else 2)
        painter.setPen(pen)
        half = 8 if selected else 6
        gap = 4
        # Two jaws
        painter.drawLine(u - half, v - gap, u - 2, v - gap)
        painter.drawLine(u - half, v + gap, u - 2, v + gap)
        painter.drawLine(u + 2, v - gap, u + half, v - gap)
        painter.drawLine(u + 2, v + gap, u + half, v + gap)
        # Center axis
        painter.drawLine(u, v - 10, u, v + 10)

    def _render_candidate_overlay(self):
        if self._latest_qimg is None:
            return
        if not self.enable_grasp_picker:
            if self.candidate_dialog.isVisible():
                pix0 = QtGui.QPixmap.fromImage(self._latest_qimg)
                self.candidate_dialog.view_label.setPixmap(
                    pix0.scaled(
                        self.candidate_dialog.view_label.size(),
                        QtCore.Qt.KeepAspectRatio,
                        QtCore.Qt.FastTransformation,
                    )
                )
            return
        now_t = rospy.get_time()
        if now_t - self._last_candidate_render_t < 0.25:
            return
        self._last_candidate_render_t = now_t
        src = self._latest_qimg.copy()
        painter = QtGui.QPainter(src)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        grasp_id = id(self._latest_grasp_msg)
        if grasp_id != self._cached_cluster_grasp_id:
            raw_ranked = []
            if self._latest_grasp_msg is not None:
                grasps_all = list(getattr(self._latest_grasp_msg, "grasps", []))
                for idx, g in enumerate(grasps_all):
                    score = 0.0
                    if hasattr(g, "score") and hasattr(g.score, "data"):
                        score = float(g.score.data)
                    raw_ranked.append((score, idx, g))
                raw_ranked.sort(key=lambda x: x[0], reverse=True)
            self._cached_clustered = _cluster_grasps(raw_ranked, max(0.005, self.cluster_radius_m)) if raw_ranked else []
            self._cached_cluster_grasp_id = grasp_id
        clustered = self._cached_clustered

        if not clustered:
            self._projection_status = self._tr("picker_no_cand")
            if self._projection_status != self._last_projection_status:
                self._last_projection_status = self._projection_status
                self.log_signal.emit("[picker] {}".format(self._projection_status))
                try:
                    rospy.loginfo("[picker] %s", self._projection_status)
                except Exception:
                    pass
            self._candidate_points_2d = []
            self._candidate_draw_meta = None
            if painter.isActive():
                painter.end()
            self.candidate_dialog.candidates_list.blockSignals(True)
            self.candidate_dialog.candidates_list.clear()
            self.candidate_dialog.candidates_list.blockSignals(False)
            pix0 = QtGui.QPixmap.fromImage(src)
            self.candidate_dialog.view_label.setPixmap(
                pix0.scaled(
                    self.candidate_dialog.view_label.size(),
                    QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.FastTransformation,
                )
            )
            return

        candidates = []
        target_frame_used = None
        source_frame_used = None
        had_any_tf = False
        had_tf_pair = None
        if self._camera_model:
            candidate_camera_frames = [norm_frame(self._camera_model.get("frame_id", ""))]
            for f in self.camera_frame_fallbacks:
                fn = norm_frame(f)
                if fn and fn not in candidate_camera_frames:
                    candidate_camera_frames.append(fn)
            # Projection must happen in optical camera frame.
            optical_camera_frames = [f for f in candidate_camera_frames if is_optical_frame(f)]
            if optical_camera_frames:
                candidate_camera_frames = optical_camera_frames
            candidate_source_frames = [norm_frame(self.grasp_points_frame)]
            try:
                msg_f = norm_frame(getattr(getattr(self._latest_grasp_msg, "header", None), "frame_id", ""))
                if msg_f and msg_f not in candidate_source_frames:
                    candidate_source_frames.append(msg_f)
            except Exception:
                pass
            # Prefer last successful pair to keep frame choice stable across rounds.
            if self._last_projection_pair:
                pref_sf, pref_cf = self._last_projection_pair
                if pref_sf in candidate_source_frames and pref_cf in candidate_camera_frames:
                    candidate_source_frames = [pref_sf] + [f for f in candidate_source_frames if f != pref_sf]
                    candidate_camera_frames = [pref_cf] + [f for f in candidate_camera_frames if f != pref_cf]

            if not getattr(self, "_tf_diag_done", False):
                all_frames = set(self._tf_edges.keys()) | set(self._tf_static_edges.keys())
                for child, rec in self._tf_static_edges.items():
                    all_frames.add(rec[0])
                for child, rec in self._tf_edges.items():
                    all_frames.add(rec[0])
                try:
                    rospy.loginfo("[picker-diag] TF frames (%d): %s", len(all_frames), sorted(all_frames))
                    rospy.loginfo("[picker-diag] candidate_camera_frames=%s  candidate_source_frames=%s", candidate_camera_frames, candidate_source_frames)
                except Exception:
                    pass
                self._tf_diag_done = True

            for sf in candidate_source_frames:
                for cf in candidate_camera_frames:
                    tf_raw = self._lookup_tf_simple(cf, sf)
                    if tf_raw is None:
                        continue
                    had_any_tf = True
                    had_tf_pair = (sf, cf)
                    tf_try = self._invert_tf(tf_raw)
                    trial = []
                    for avg_score, best_idx, ax, ay, az in clustered:
                        uv = self._project_point_to_image(ax, ay, az, tf_try)
                        if uv is not None:
                            trial.append({"idx": best_idx, "u": uv[0], "v": uv[1], "score": avg_score})
                    if not trial and not getattr(self, "_proj_sample_done", False):
                        try:
                            if clustered:
                                _s0, _i0, px0, py0, pz0 = clustered[0]
                                tx, ty, tz, qx, qy, qz, qw = tf_try
                                r = quat_to_rot(qx, qy, qz, qw)
                                cx = r[0][0]*px0 + r[0][1]*py0 + r[0][2]*pz0 + tx
                                cy = r[1][0]*px0 + r[1][1]*py0 + r[1][2]*pz0 + ty
                                cz = r[2][0]*px0 + r[2][1]*py0 + r[2][2]*pz0 + tz
                                cam = self._camera_model
                                rospy.loginfo("[picker-diag] sf=%s cf=%s  cluster0=(%.4f,%.4f,%.4f) -> cam=(%.4f,%.4f,%.4f) cz=%.4f  cam_wh=(%d,%d) fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                                    sf, cf, px0, py0, pz0, cx, cy, cz, cz, cam.get("width",0), cam.get("height",0), cam.get("fx",0), cam.get("fy",0), cam.get("cx",0), cam.get("cy",0))
                                if cz > 1e-6:
                                    u = cam["fx"]*(cx/cz)+cam["cx"]
                                    v = cam["fy"]*(cy/cz)+cam["cy"]
                                    rospy.loginfo("[picker-diag] -> pixel=(%.1f,%.1f) in_bounds=%s", u, v, (0<=u<cam["width"] and 0<=v<cam["height"]))
                            self._proj_sample_done = True
                        except Exception as e:
                            rospy.logwarn("[picker-diag] err: %s", e)
                    if trial:
                        candidates = trial
                        source_frame_used = sf
                        target_frame_used = cf
                        self._last_projection_pair = (sf, cf)
                        break
                if candidates:
                    break
        if candidates:
            self._projection_status = self._tr("proj_ok").format(
                source_frame_used or "?",
                target_frame_used or "?",
            )
        elif had_any_tf:
            sf, cf = had_tf_pair if had_tf_pair else ("?", "?")
            self._projection_status = self._tr("proj_fail_px").format(sf, cf)
        else:
            self._projection_status = self._tr("proj_fail_tf").format(
                norm_frame(self.grasp_points_frame),
                norm_frame(self._camera_model.get("frame_id", "")) if self._camera_model else "camera",
            )
        if self._projection_status != self._last_projection_status:
            self._last_projection_status = self._projection_status
            self.log_signal.emit("[picker] {}".format(self._projection_status))
            try:
                rospy.loginfo("[picker] %s", self._projection_status)
            except Exception:
                pass

        now_t2 = rospy.get_time()
        if candidates:
            self._projected_candidates_cache = list(candidates)
            self._projected_candidates_cache_t = now_t2
        elif now_t2 - self._projected_candidates_cache_t < 2.0:
            candidates = list(self._projected_candidates_cache)

        signature = tuple((int(bi), round(float(sc), 4)) for sc, bi, _x, _y, _z in clustered)
        if signature != self._candidate_list_signature or self._candidate_list_selected != self._selected_candidate_index:
            self.candidate_dialog.candidates_list.blockSignals(True)
            self.candidate_dialog.candidates_list.clear()
            for rank, (avg_score, best_idx, _x, _y, _z) in enumerate(clustered, start=1):
                item = QtWidgets.QListWidgetItem("C{}  #{}  score={:.3f}".format(rank, best_idx, avg_score))
                item.setData(QtCore.Qt.UserRole, int(best_idx))
                if best_idx == self._selected_candidate_index:
                    item.setBackground(QtGui.QColor(70, 70, 20))
                self.candidate_dialog.candidates_list.addItem(item)
            self.candidate_dialog.candidates_list.blockSignals(False)
            self._candidate_list_signature = signature
            self._candidate_list_selected = self._selected_candidate_index

        for c in candidates:
            is_sel = c["idx"] == self._selected_candidate_index
            color = QtGui.QColor(255, 220, 0) if is_sel else QtGui.QColor(0, 255, 160)
            self._draw_gripper_marker(painter, c["u"], c["v"], color, is_sel)
            painter.setPen(QtGui.QPen(color))
            painter.drawText(c["u"] + 8, c["v"] - 8, "#{} {:.2f}".format(c["idx"], c["score"]))

        if (not candidates) and clustered:
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 200, 80)))
            painter.drawText(16, 28, self._tr("proj_fallback").format(self._projection_status))
            base_y = 56
            for row, (avg_score, best_idx, _x, _y, _z) in enumerate(clustered[:8]):
                y = base_y + row * 28
                is_sel = best_idx == self._selected_candidate_index
                color = QtGui.QColor(255, 220, 0) if is_sel else QtGui.QColor(0, 255, 160)
                self._draw_gripper_marker(painter, 24, y, color, is_sel)
                painter.setPen(QtGui.QPen(color))
                painter.drawText(42, y + 5, "C{} #{} {:.2f}".format(row + 1, best_idx, avg_score))
        if painter.isActive():
            painter.end()
        self._candidate_points_2d = candidates

        pix = QtGui.QPixmap.fromImage(src)
        scaled = pix.scaled(
            self.candidate_dialog.view_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        self._candidate_draw_meta = {
            "src_w": src.width(),
            "src_h": src.height(),
            "draw_w": scaled.width(),
            "draw_h": scaled.height(),
            "off_x": int((self.candidate_dialog.view_label.width() - scaled.width()) / 2),
            "off_y": int((self.candidate_dialog.view_label.height() - scaled.height()) / 2),
        }
        self.candidate_dialog.view_label.setPixmap(scaled)
        self.candidate_dialog.selected_label.setText(
            self._tr("sel_auto") if self._selected_candidate_index < 0
            else self._tr("sel_fmt").format(self._selected_candidate_index)
        )

    def _on_candidate_view_clicked(self, x, y):
        if not self.enable_grasp_picker:
            return
        if not self._candidate_points_2d or not self._candidate_draw_meta:
            return
        m = self._candidate_draw_meta
        if x < m["off_x"] or y < m["off_y"] or x >= (m["off_x"] + m["draw_w"]) or y >= (m["off_y"] + m["draw_h"]):
            return
        img_x = (float(x - m["off_x"]) * float(m["src_w"])) / max(1.0, float(m["draw_w"]))
        img_y = (float(y - m["off_y"]) * float(m["src_h"])) / max(1.0, float(m["draw_h"]))
        best = None
        best_d2 = 1e18
        for c in self._candidate_points_2d:
            du = float(c["u"]) - img_x
            dv = float(c["v"]) - img_y
            d2 = du * du + dv * dv
            if d2 < best_d2:
                best_d2 = d2
                best = c
        if best is None:
            return
        self._selected_candidate_index = int(best["idx"])
        try:
            rospy.set_param("/gpd_grasp/selected_grasp_index", int(self._selected_candidate_index))
        except Exception:
            pass
        self.log_signal.emit("已选择候选夹子 #{}".format(self._selected_candidate_index))
        self._render_candidate_overlay()

    def _on_candidate_list_clicked(self, item):
        if not self.enable_grasp_picker:
            return
        idx = item.data(QtCore.Qt.UserRole)
        if idx is None:
            return
        self._selected_candidate_index = int(idx)
        try:
            rospy.set_param("/gpd_grasp/selected_grasp_index", int(self._selected_candidate_index))
        except Exception:
            pass
        self.log_signal.emit("已从列表选择候选夹子 #{}".format(self._selected_candidate_index))
        self._render_candidate_overlay()

    def _confirm_candidate_selection(self):
        if not self.enable_grasp_picker:
            return
        if self._selected_candidate_index < 0:
            self.log_signal.emit("请先选择一个候选夹子再确认")
            return
        self._selection_confirmed = True
        self._selection_event.set()
        if self._selection_waiting:
            self.log_signal.emit("已确认候选 #{}，准备触发抓取".format(self._selected_candidate_index))
        else:
            self.log_signal.emit("已确认候选 #{}（等待下一次抓取）".format(self._selected_candidate_index))

    def _clear_selected_candidate(self):
        if not self.enable_grasp_picker:
            return
        self._selected_candidate_index = -1
        self._selection_confirmed = False
        try:
            rospy.set_param("/gpd_grasp/selected_grasp_index", -1)
        except Exception:
            pass
        self.log_signal.emit("已清除候选选择，恢复自动抓取")
        self._render_candidate_overlay()

    def _read_ui_config(self):
        try:
            self.ui_config.load()
            return self.ui_config.data or {}
        except Exception:
            return {}

    def _on_store_clicked(self):
        cfg = self._read_ui_config()
        pose = ((cfg.get("poses") or {}).get("grab") or {})
        joints = pose.get("joints", [])
        duration_ms = int(pose.get("duration_ms", 800))
        real_duration_ms = self._scaled_duration_ms(cfg, duration_ms)
        if not joints:
            self.log_signal.emit("存放失败: ui_config 里 poses.grab 为空")
            return
        self.log_signal.emit("执行存放姿态: poses.grab")
        self._publish_servo_pose(real_duration_ms, joints)
        QtCore.QTimer.singleShot(real_duration_ms + 150, self._on_release_clicked)

    def _on_release_clicked(self):
        self.log_signal.emit("执行释放: 打开夹爪到 {}".format(self.gripper_open_position))
        self._publish_servo_pose(self.gripper_duration_ms, [[self.gripper_servo_id, self.gripper_open_position]])
        self._schedule_return_init()

    def _schedule_return_init(self):
        delay_ms = int(max(0.0, self.auto_return_init_delay_s) * 1000.0)
        self.log_signal.emit("{} ms 后自动回初始姿态".format(delay_ms))
        QtCore.QTimer.singleShot(delay_ms, lambda: self._move_pose_from_ui_config("init"))

    def _update_image(self, qimg: QtGui.QImage):
        self._latest_qimg = qimg.copy()
        self.video_label.setPixmap(
            QtGui.QPixmap.fromImage(qimg).scaled(
                self.video_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation
            )
        )
        self._render_candidate_overlay()

    def _update_depth_image(self, qimg: QtGui.QImage):
        self.depth_label.setPixmap(
            QtGui.QPixmap.fromImage(qimg).scaled(
                self.depth_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation
            )
        )

    def _apply_to_widgets(self):
        for name, (slider, spin, vmin, step) in self.controls.items():
            val = float(self.values.get(name, 0.0))
            spin.blockSignals(True)
            slider.blockSignals(True)
            spin.setValue(val)
            slider.setValue(int(round((val - vmin) / step)))
            slider.blockSignals(False)
            spin.blockSignals(False)
        for name, (slider, spin, vmin, step) in self.grasp_controls.items():
            val = float(self.grasp_offset.get(name, 0.0))
            spin.blockSignals(True)
            slider.blockSignals(True)
            spin.setValue(val)
            slider.setValue(int(round((val - vmin) / step)))
            slider.blockSignals(False)
            spin.blockSignals(False)
        self.parent_edit.setText(self.parent_frame)
        self.child_edit.setText(self.child_frame)
        self._apply_motion_widgets()
        self.grab_delay_spin.blockSignals(True)
        self.grab_delay_spin.setValue(float(self.grab_delay_before_scan_s))
        self.grab_delay_spin.blockSignals(False)
        self.attack_angle_spin.blockSignals(True)
        self.approach_distance_spin.blockSignals(True)
        self.edge_margin_spin.blockSignals(True)
        self.edge_penalty_spin.blockSignals(True)
        self.attack_angle_spin.setValue(float(self.attack_angle_deg))
        self.approach_distance_spin.setValue(float(self.approach_distance_m))
        self.edge_margin_spin.setValue(float(self.edge_margin_m))
        self.edge_penalty_spin.setValue(float(self.edge_penalty_weight))
        self.edge_penalty_spin.blockSignals(False)
        self.edge_margin_spin.blockSignals(False)
        self.approach_distance_spin.blockSignals(False)
        self.attack_angle_spin.blockSignals(False)
        self.picker_offset_u_spin.blockSignals(True)
        self.picker_offset_v_spin.blockSignals(True)
        self.cluster_radius_spin.blockSignals(True)
        self.picker_offset_u_spin.setValue(int(self.picker_offset_u))
        self.picker_offset_v_spin.setValue(int(self.picker_offset_v))
        self.cluster_radius_spin.setValue(float(self.cluster_radius_m))
        self.cluster_radius_spin.blockSignals(False)
        self.picker_offset_v_spin.blockSignals(False)
        self.picker_offset_u_spin.blockSignals(False)
        saved_lang = str(self.ui_state.value("language", "en") or "en")
        if saved_lang != self._lang:
            self._lang = saved_lang
            self._retranslate_ui()
        self._update_preview()

    def _on_speed_slider_changed(self, value):
        self.speed_spin.blockSignals(True)
        self.speed_spin.setValue(int(value))
        self.speed_spin.blockSignals(False)
        self.motion_speed_percent = int(value)
        self._update_preview()

    def _on_speed_spin_changed(self, value):
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value))
        self.speed_slider.blockSignals(False)
        self.motion_speed_percent = int(value)
        self._update_preview()

    def _apply_speed_preset(self, speed_percent):
        self.motion_speed_percent = int(max(30, min(120, speed_percent)))
        self._apply_motion_widgets()
        self._save_motion_config()

    def _apply_motion_widgets(self):
        self.speed_slider.blockSignals(True)
        self.speed_spin.blockSignals(True)
        self.speed_slider.setValue(int(self.motion_speed_percent))
        self.speed_spin.setValue(int(self.motion_speed_percent))
        self.speed_spin.blockSignals(False)
        self.speed_slider.blockSignals(False)

    def _on_grab_delay_changed(self, value):
        self.grab_delay_before_scan_s = float(value)
        self._update_preview()

    def _on_attack_angle_changed(self, value):
        self.attack_angle_deg = float(value)
        self._apply_gpd_tuning_params()
        self._update_preview()

    def _on_approach_distance_changed(self, value):
        self.approach_distance_m = float(value)
        self._apply_gpd_tuning_params()
        self._update_preview()

    def _on_edge_margin_changed(self, value):
        self.edge_margin_m = float(value)
        self._apply_gpd_tuning_params()
        self._update_preview()

    def _on_edge_penalty_changed(self, value):
        self.edge_penalty_weight = float(value)
        self._apply_gpd_tuning_params()
        self._update_preview()

    def _on_picker_offset_u_changed(self, value):
        self.picker_offset_u = int(value)
        self._projected_candidates_cache = []
        self._proj_sample_done = False

    def _on_picker_offset_v_changed(self, value):
        self.picker_offset_v = int(value)
        self._projected_candidates_cache = []
        self._proj_sample_done = False

    def _apply_gpd_tuning_params(self):
        try:
            rospy.set_param("/gpd_grasp/attack_angle_deg", float(self.attack_angle_deg))
            rospy.set_param("/gpd_grasp/approach_distance_m", float(self.approach_distance_m))
            rospy.set_param("/gpd_grasp/edge_margin_m", float(self.edge_margin_m))
            rospy.set_param("/gpd_grasp/edge_penalty_weight", float(self.edge_penalty_weight))
        except Exception:
            pass

    def _save_motion_config(self):
        cfg = self._read_ui_config()
        motion = cfg.get("motion", {}) if isinstance(cfg.get("motion"), dict) else {}
        motion["speed_percent"] = int(self.motion_speed_percent)
        cfg["motion"] = motion
        self.ui_config.data = cfg
        self.ui_config.save()
        save_path = self.ui_config.user_path if getattr(self.ui_config, "user_path", None) else self.ui_config.path
        self.log_signal.emit(
            "已保存速度到 {}: {}%".format(save_path, int(self.motion_speed_percent))
        )

    def _reload_all_config(self):
        self._load_tf_config()
        self._apply_to_widgets()
        self.log_signal.emit("已重新加载配置")

    def _load_tf_config(self):
        read_path = self.config_path
        if not os.path.exists(read_path) and os.path.exists(self.legacy_tf_config_path):
            read_path = self.legacy_tf_config_path
            self.log_signal.emit("使用旧路径加载TF配置: {}".format(read_path))
        if not os.path.exists(read_path):
            return
        try:
            with open(read_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
            self.parent_frame = cfg.get("parent_frame", self.parent_frame)
            self.child_frame = cfg.get("child_frame", self.child_frame)
            tfv = cfg.get("transform", {})
            for key in self.values:
                if key in tfv:
                    self.values[key] = float(tfv[key])
            go = cfg.get("grasp_offset", {})
            for key in self.grasp_offset:
                if key in go:
                    self.grasp_offset[key] = float(go[key])
            if "grab_delay_before_scan_s" in cfg:
                self.grab_delay_before_scan_s = float(cfg.get("grab_delay_before_scan_s", self.grab_delay_before_scan_s))
            gpd_tuning = cfg.get("gpd_tuning", {}) if isinstance(cfg.get("gpd_tuning"), dict) else {}
            if "attack_angle_deg" in gpd_tuning:
                self.attack_angle_deg = float(gpd_tuning.get("attack_angle_deg", self.attack_angle_deg))
            if "approach_distance_m" in gpd_tuning:
                self.approach_distance_m = float(
                    gpd_tuning.get("approach_distance_m", self.approach_distance_m)
                )
            if "edge_margin_m" in gpd_tuning:
                self.edge_margin_m = float(gpd_tuning.get("edge_margin_m", self.edge_margin_m))
            if "edge_penalty_weight" in gpd_tuning:
                self.edge_penalty_weight = float(gpd_tuning.get("edge_penalty_weight", self.edge_penalty_weight))
            if "picker_offset_u" in gpd_tuning:
                self.picker_offset_u = int(gpd_tuning.get("picker_offset_u", self.picker_offset_u))
            if "picker_offset_v" in gpd_tuning:
                self.picker_offset_v = int(gpd_tuning.get("picker_offset_v", self.picker_offset_v))
            if "cluster_radius_m" in gpd_tuning:
                self.cluster_radius_m = float(gpd_tuning.get("cluster_radius_m", self.cluster_radius_m))
        except Exception as exc:
            self.log_signal.emit("加载配置失败: {}".format(str(exc)))
        cfg = self._read_ui_config()
        motion = cfg.get("motion", {}) if isinstance(cfg.get("motion"), dict) else {}
        self.motion_speed_percent = int(max(30, min(120, motion.get("speed_percent", 100))))
        self._apply_gpd_tuning_params()

    def _save_all_config(self):
        self.parent_frame = self.parent_edit.text().strip() or self.parent_frame
        self.child_frame = self.child_edit.text().strip() or self.child_frame
        data = {
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "transform": {k: float(v) for k, v in self.values.items()},
            "grasp_offset": {k: float(v) for k, v in self.grasp_offset.items()},
            "grab_delay_before_scan_s": float(self.grab_delay_before_scan_s),
            "gpd_tuning": {
                "attack_angle_deg": float(self.attack_angle_deg),
                "approach_distance_m": float(self.approach_distance_m),
                "edge_margin_m": float(self.edge_margin_m),
                "edge_penalty_weight": float(self.edge_penalty_weight),
                "picker_offset_u": int(self.picker_offset_u),
                "picker_offset_v": int(self.picker_offset_v),
                "cluster_radius_m": float(self.cluster_radius_m),
            },
        }
        cfg_dir = os.path.dirname(self.config_path)
        if cfg_dir and not os.path.exists(cfg_dir):
            os.makedirs(cfg_dir)
        with open(self.config_path, "w") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        self.log_signal.emit("已保存: {}".format(self.config_path))
        self._update_preview()

    def _update_preview(self):
        p = self.parent_edit.text().strip() or self.parent_frame
        c = self.child_edit.text().strip() or self.child_frame
        x = self.values["x"]
        y = self.values["y"]
        z = self.values["z"]
        r = self.values["roll"]
        pit = self.values["pitch"]
        yw = self.values["yaw"]
        gox = self.grasp_offset["x"]
        goy = self.grasp_offset["y"]
        goz = self.grasp_offset["z"]
        self.preview.setPlainText(
            "实时TF: {} -> {}\n"
            "xyz: {:.3f} {:.3f} {:.3f}\n"
            "rpy: {:.3f} {:.3f} {:.3f}\n\n"
            "grasp_offset: {:.3f} {:.3f} {:.3f}\n"
            "grab_delay_before_scan: {:.1f}s\n"
            "motion_speed: {}%\n"
            "attack_angle: {:.1f} deg\n"
            "approach_distance: {:.3f} m\n"
            "edge_margin: {:.3f} m\n"
            "edge_penalty_weight: {:.2f}\n"
            "picker_top_k: {}\n"
            "projection: {}\n"
            "scan_mode: {}\n"
            "wait_topic: {}\n"
            "once_service: {}\n"
            "trigger_service: {}\n".format(
                p,
                c,
                x,
                y,
                z,
                r,
                pit,
                yw,
                gox,
                goy,
                goz,
                float(self.grab_delay_before_scan_s),
                int(self.motion_speed_percent),
                float(self.attack_angle_deg),
                float(self.approach_distance_m),
                float(self.edge_margin_m),
                float(self.edge_penalty_weight),
                int(self.candidate_top_k),
                self._projection_status,
                "single-shot" if self.use_single_shot_scan else "stream",
                self.grasp_topic,
                self.single_shot_gpd_service,
                self.grasp_trigger_service,
            )
        )

    def _publish_tf(self):
        parent = self.parent_edit.text().strip() or self.parent_frame
        child = self.child_edit.text().strip() or self.child_frame
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(self.values["x"])
        t.transform.translation.y = float(self.values["y"])
        t.transform.translation.z = float(self.values["z"])
        qx, qy, qz, qw = rpy_to_quat(
            float(self.values["roll"]),
            float(self.values["pitch"]),
            float(self.values["yaw"]),
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage([t]))

    def closeEvent(self, event):
        try:
            rospy.set_param("/gpd_grasp/selected_grasp_index", -1)
            self._save_candidate_dialog_geometry()
            self._stop_required_services()
            self.settings_dialog.hide()
            self.candidate_dialog.hide()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = TfCalibrationControlUi()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
