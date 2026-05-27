#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RRR GUI Controller v5
Real 하드웨어 제어 + Isaac Sim 연동 + Isaac Sim
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
import json
import math
import os
import subprocess
import sys
import socket
import cv2
import numpy as np
from datetime import datetime
from pathlib import Path

# ========== 프로젝트 루트 자동 감지 (상대경로) ==========
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))  # 1_Robot_GUI/
_GUI_ROOT = os.path.dirname(_THIS_DIR)                   # RRR_GUI/
_PROJECT_ROOT = os.path.dirname(_GUI_ROOT)                # RRR_Standalone/
try:
    import h5py
    H5PY_AVAILABLE = True
except ImportError:
    H5PY_AVAILABLE = False
    print("Warning: h5py not installed. LeRobot conversion disabled.")

# OpenVR (SteamVR) for VR Teleoperation
try:
    import openvr
    OPENVR_AVAILABLE = True
except ImportError:
    OPENVR_AVAILABLE = False
    print("Warning: openvr not installed. VR teleoperation disabled.")

class RobotController:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RRR Controller v5 - Real Hardware + Sim")
        self.root.geometry("2080x1170")
        self.root.configure(bg='#2c3e50')

        # ========== 시스템 모니터링 ==========
        self.system_monitor_running = True
        self.cpu_usage = 0
        self.ram_usage = 0
        self.gpu_usage = 0
        self.gpu_mem = 0
        self.temp_cpu = 0
        self.temp_gpu = 0

        # ========== Real 모드 변수 ==========
        self.real_ser = None
        self.real_connected = False
        self.real_active_joints = {}
        self.real_sliders = {}
        self.real_value_labels = {}
        self.real_baseline_positions = {}
        self.invert_buttons = {}  # Sim 반전 버튼 저장
        self.real_time_enabled = tk.BooleanVar(value=True)
        self.real_acceleration_value = tk.IntVar(value=6)  # ACC 기본값

        # ========== Sim 연동 변수 ==========
        self.sync_mode = tk.StringVar(value="독립")  # "독립", "Real->Sim", "Sim->Real"
        self.real_link_to_sim = False  # Real 슬라이더가 Sim을 제어하는지
        self.sim_link_to_real = False  # Sim이 Real을 제어하는지
        self.updating_from_sim = False  # Sim->Real 업데이트 중 플래그 (무한 루프 방지)
        self.sim_connected = False
        self.sim_active_joints = {}
        self.sim_baseline_positions = {}
        self.sim_command_file = os.path.join(_GUI_ROOT, "ros_files", "isaac_sim_integration", "sim_commands_v4.json")
        self.sim_process = None
        self.sim_velocity_value = tk.DoubleVar(value=1.0)  # Sim 속도 (rad/s)
        self.sim_inverted = {}  # Sim 반전 상태 저장 (servo_id: bool)
        self.loading_baseline = False  # 베이스라인 로드 중 플래그
        self._last_sim_update_time = 0  # update_sim 스로틀링용
        self._sim_update_pending = False  # 대기 중인 sim 업데이트

        # ========== 서보 위치 읽기(Read-back) 변수 ==========
        self.readback_running = False
        self.readback_interval = 100  # ms (초당 10회)

        # ========== 네트워크 동기화 변수 ==========
        self.network_client_socket = None
        self.network_connected = False
        self.network_running = False
        self.network_port = 9999
        self.network_server_ip = "100.90.7.84"  # Windows Tailscale IP
        self.network_recv_thread = None
        self.network_last_send_time = 0
        self.network_send_interval = 0.05
        self._network_updating = False

        # ========== IK (Inverse Kinematics) 변수 - XYZ만 ==========
        self.ik_enabled = tk.BooleanVar(value=False)
        self.ik_sliders = {'right': {}, 'left': {}}  # x, y, z 슬라이더
        self.ik_value_labels = {'right': {}, 'left': {}}  # 값 표시 라벨
        # IK 슬라이더 값 = baseline에서의 delta (0 = baseline)
        self.ik_values = {
            'right': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'left': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        # VR 방향 델타 (roll, pitch, yaw in radians)
        self.ik_orientation_delta = {
            'right': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'left': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
        # baseline XYZ 위치 (FK로 계산)
        self.ik_baseline_xyz = {
            'right': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'left': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        # baseline RPY 방향 (FK로 계산) - 6-DOF IK용
        self.ik_baseline_rpy = {
            'right': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'left': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
        self.ik_solver = {'right': None, 'left': None}  # 팔별 IK 솔버
        self._ik_updating = False  # IK↔서보 업데이트 중 플래그
        self._ik_update_pending = False  # IK 계산 스로틀링용

        # ========== VR Teleoperation Variables ==========
        self.vr_enabled = tk.BooleanVar(value=False)
        self.vr_system = None  # OpenVR system
        self.vr_running = False
        self.vr_thread = None
        self.vr_scale = tk.DoubleVar(value=500.0)  # VR 움직임 스케일 (mm)
        self.vr_deadzone = 0.005  # 데드존 (m)
        # VR 컨트롤러 초기 위치 (캘리브레이션용)
        self.vr_origin = {'right': None, 'left': None}
        # VR 그리퍼 상태
        self.vr_gripper = {'right': 0, 'left': 0}
        # VR IK 캐시 (리밋 시 이전 유효값 사용)
        self.vr_last_valid_servos = {'right': None, 'left': None}
        self.vr_last_valid_xyz = {'right': None, 'left': None}
        # VR IK 펜딩 콜백 ID (이전 콜백 취소용)
        self.vr_ik_pending = {'right': None, 'left': None}
        # VR 팔 선택 (all, left, right)
        self.vr_arm_select = tk.StringVar(value="all")

        # ========== Wheel Control Variables ==========
        self.wheel_speed = tk.DoubleVar(value=2.0)  # 바퀴 속도 (rad/s)
        self.wheel_command_file = os.path.join(_GUI_ROOT, "isaac_sim_integration", "wheel_commands_v5.json")
        # Button press states for simultaneous control
        self.btn_up_pressed = False
        self.btn_down_pressed = False
        self.btn_left_pressed = False
        self.btn_right_pressed = False

        # ========== Lift Control Variables ==========
        self.lift_speed = 0.1  # m/s
        self.lift_position = 0.0  # Current position (0 = max, -0.5 = min)
        self.lift_min = -0.8  # Minimum position (80cm down)
        self.lift_max = 0.0   # Maximum position (current)
        self.lift_command_file = os.path.join(_GUI_ROOT, "isaac_sim_integration", "lift_commands_v5.json")
        self.lift_q_pressed = False
        self.lift_a_pressed = False
        self.lift_update_running = False  # 업데이트 루프 실행 중 플래그
        self.lift_last_key_time = 0  # 마지막 키 입력 시간 (디바운스용)
        self.slider_scale = 0.9  # 슬라이더 크기 배율 (기본 90%)

        # ========== 데이터 수집 변수 (Pi0.5 파인튜닝용 - LeRobot v2 포맷) ==========
        self.base_data_dir = os.path.join(_GUI_ROOT, "datasets")
        self.current_dataset_name = tk.StringVar(value="rx1_teleop_v1")
        self.current_task_name = tk.StringVar(value="pick_and_place")
        self.language_instruction = tk.StringVar(value="pick up the object")
        self.recording_date = None  # 녹화 시작 날짜 (YYYY_MM_DD)

        # 카메라 (Pi0.5: observation.images.cam_high 등)
        self.data_cameras = {}  # {cam_number: capture_object}
        self.camera_frames = {}  # {cam_number: latest_frame}
        self.camera_canvases = {}  # {cam_number: canvas_widget}
        self.camera_labels = {}  # {cam_number: label_widget}
        self.stop_camera = False
        self.camera_thread = None
        self._camera_display_active = False
        self.detected_cameras = []  # 감지된 카메라 목록
        self.cam_grid = None  # 카메라 그리드 프레임

        # 카메라 멈춤 감지용
        self.camera_last_hash = {}  # {cam_num: hash}
        self.camera_last_change_time = {}  # {cam_num: time}
        self.camera_freeze_warning = None  # 경고 라벨 위젯
        self.camera_freeze_threshold = 2.0  # 2초 이상 변화 없으면 경고

        # Pi0.5 카메라 키 매핑 (카메라 번호 → Pi0.5 observation 키)
        # cam_1: Gemini2 탑뷰, cam_2: C270 우손목, cam_3: C270 좌손목
        self.pi0_camera_keys = {
            'cam_1': 'observation.images.top',           # 1번 → 탑뷰
            'cam_2': 'observation.images.wrist_right',   # 2번 → 우손목
            'cam_3': 'observation.images.wrist_left',    # 3번 → 좌손목
        }

        # Orbbec SDK 관련
        self.orbbec_pipeline = None
        self.orbbec_available = False
        try:
            from pyorbbecsdk import Pipeline, Config, OBSensorType
            self.orbbec_available = True
        except ImportError:
            pass

        # 녹화 상태
        self.is_recording = False
        self.current_episode = []
        self.episode_count = self._get_next_episode_number()  # 기존 에피소드 다음 번호
        self.frame_count = 0
        self.recording_fps = 11
        self.recording_start_time = None
        self.keyboard_recording_enabled = False  # 키보드 녹화 컨트롤 토글
        self.teleop_baseline_locked = True  # 텔레옵 연동 시 기본자세 잠금 (녹화 전까지)

        # 학습용 관절 ID (16 DOF) - Pi0.5는 최대 18차원 지원
        # RX-1: 오른팔 7 + 그리퍼 1 + 왼팔 7 + 그리퍼 1 = 16 DOF
        self.learning_joint_ids = [
            11, 12, 13, 14, 15, 16, 17, 41,  # 오른팔 7 DOF + 그리퍼 1
            21, 22, 23, 24, 25, 26, 27, 31,  # 왼팔 7 DOF + 그리퍼 1
        ]

        # Pi0.5 정규화 범위 (서보 0-4095 -> -1 to 1)
        self.servo_min = 0
        self.servo_max = 4095
        self.servo_center = 2048

        # ========== 디버그 로그 파일 ==========
        self.debug_log_file = os.path.join(_THIS_DIR, "debug_log.txt")
        self.init_debug_log()

        # ========== OpenVR (SteamVR) 텔레옵 ==========
        self.vr_enabled = False
        self.vr_system = None  # OpenVR system
        self.vr_thread = None
        self.vr_scale = 1.5  # VR 움직임 스케일 (1.5 = 150%)
        self.vr_calibrated = {'right': False, 'left': False}
        self.vr_base_pose = {'right': None, 'left': None}
        self.vr_offset = {'right': [0.0, 0.0, 0.0], 'left': [0.0, 0.0, 0.0]}
        self.vr_base_orientation = {'right': None, 'left': None}  # 캘리브레이션 시 VR 회전
        self.vr_rotation_offset = {'right': [0.0, 0.0, 0.0], 'left': [0.0, 0.0, 0.0]}  # 로봇 손목 기준
        self.vr_controller_ids = {'right': None, 'left': None}

        # 서보 정의
        self.init_real_joints()
        self.init_sim_joints()
        self.init_teleop_joints()

        # UI 생성
        self.create_ui()

        # 베이스라인 로드
        self.load_real_baseline()
        self.load_sim_baseline()

        # IK 슬라이더 초기값을 서보 FK에서 계산 (오류 시 무시)
        try:
            self._init_ik_sliders_from_servo()
        except Exception as e:
            print(f"IK 초기화 실패 (무시): {e}")

        # Sim Velocity 변경 시 Real ACC 자동 연동
        self.sim_velocity_value.trace_add("write", self.on_sim_velocity_change)


    def init_debug_log(self):
        """디버그 로그 파일 초기화"""
        try:
            with open(self.debug_log_file, 'w', encoding='utf-8') as f:
                f.write("=" * 80 + "\n")
                f.write("RRR Controller v5 Debug Log\n")
                f.write(f"Started at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 80 + "\n\n")
        except Exception as e:
            print(f"디버그 로그 파일 초기화 실패: {e}")

    def debug_log(self, message):
        """디버그 로그를 파일에 저장"""
        try:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with open(self.debug_log_file, 'a', encoding='utf-8') as f:
                f.write(f"[{timestamp}] {message}\n")
        except Exception as e:
            pass

    def init_real_joints(self):
        """Real 모드 서보 초기화"""
        self.real_active_joints = {
            11: {"name": "R Shoulder F/B", "current": 4000, "min": 0, "max": 4095},
            12: {"name": "R Shoulder U/D", "current": 1400, "min": 0, "max": 4095},
            13: {"name": "R Arm Rot", "current": 3000, "min": 0, "max": 4095},
            14: {"name": "R Elbow", "current": 505, "min": 0, "max": 4095},
            15: {"name": "R Wrist Rot", "current": 2825, "min": 1953, "max": 4095},  # 하드웨어 제한
            16: {"name": "R Wrist F/B", "current": 1800, "min": 1600, "max": 2400},  # 하드웨어 제한
            17: {"name": "R Wrist L/R", "current": 2680, "min": 2250, "max": 3500},  # 하드웨어 제한
            41: {"name": "R Gripper Tip1", "current": 10, "min": 10, "max": 4035},
            32: {"name": "R Gripper Tip2", "current": 10, "min": 10, "max": 4035},
            21: {"name": "L Shoulder F/B", "current": 95, "min": 0, "max": 4095},
            22: {"name": "L Shoulder U/D", "current": 2800, "min": 0, "max": 4095},
            23: {"name": "L Arm Rot", "current": 1095, "min": 0, "max": 4095},
            24: {"name": "L Elbow", "current": 3600, "min": 0, "max": 4095},
            25: {"name": "L Wrist Rot", "current": 2225, "min": 1125, "max": 3214},  # 하드웨어 제한
            26: {"name": "L Wrist F/B", "current": 2350, "min": 1654, "max": 2481},  # 하드웨어 제한
            27: {"name": "L Wrist L/R", "current": 1839, "min": 1342, "max": 2373},  # 하드웨어 제한
            31: {"name": "L Gripper Tip1", "current": 10, "min": 10, "max": 4035},
            28: {"name": "L Gripper Tip2", "current": 10, "min": 10, "max": 4035},
        }

    def init_sim_joints(self):
        """Sim 모드 서보 초기화 (Isaac Sim 연동용)"""
        # 베이스라인과 동일한 값으로 초기화
        self.sim_active_joints = {
            11: {"name": "R Shoulder F/B", "current": 4000},
            12: {"name": "R Shoulder U/D", "current": 1400},
            13: {"name": "R Arm Rot", "current": 3000},
            14: {"name": "R Elbow", "current": 505},
            15: {"name": "R Wrist Rot", "current": 2825},
            16: {"name": "R Wrist F/B", "current": 1800},
            17: {"name": "R Wrist L/R", "current": 2680},
            41: {"name": "R Gripper Tip1", "current": 0},
            32: {"name": "R Gripper Tip2", "current": 0},
            21: {"name": "L Shoulder F/B", "current": 95},
            22: {"name": "L Shoulder U/D", "current": 2800},
            23: {"name": "L Arm Rot", "current": 1095},
            24: {"name": "L Elbow", "current": 3600},
            25: {"name": "L Wrist Rot", "current": 2225},
            26: {"name": "L Wrist F/B", "current": 2350},
            27: {"name": "L Wrist L/R", "current": 1839},
            31: {"name": "L Gripper Tip1", "current": 0},
            28: {"name": "L Gripper Tip2", "current": 0},
        }

    def init_teleop_joints(self):
        """텔레오퍼레이션 서보 초기화"""
        # 텔레옵 ID -> 로봇 ID 매핑
        self.teleop_to_robot_mapping = {
            51: 11, 52: 12, 53: 13, 54: 14, 55: 15, 56: 16, 57: 17, 58: 41,  # 오른팔
            61: 21, 62: 22, 63: 23, 64: 24, 65: 25, 66: 26, 67: 27, 68: 31,  # 왼팔
        }

        # 텔레옵 조인트 정의 (기본자세 포함)
        self.teleop_active_joints = {
            # 오른팔 (5x 시리즈)
            51: {"name": "R Shoulder F/B", "current": 2964, "min": 0, "max": 4095, "robot_id": 11},
            52: {"name": "R Shoulder U/D", "current": 92, "min": 0, "max": 4095, "robot_id": 12},
            53: {"name": "R Arm Rot", "current": 1923, "min": 0, "max": 4095, "robot_id": 13},
            54: {"name": "R Elbow", "current": 4095, "min": 0, "max": 4095, "robot_id": 14},
            55: {"name": "R Wrist Rot", "current": 2998, "min": 0, "max": 4095, "robot_id": 15},
            56: {"name": "R Wrist F/B", "current": 2338, "min": 0, "max": 4095, "robot_id": 16},
            57: {"name": "R Wrist L/R", "current": 2978, "min": 0, "max": 4095, "robot_id": 17},
            58: {"name": "R Gripper", "current": 10, "min": 10, "max": 4035, "robot_id": 41},
            # 왼팔 (6x 시리즈)
            61: {"name": "L Shoulder F/B", "current": 1241, "min": 0, "max": 4095, "robot_id": 21},
            62: {"name": "L Shoulder U/D", "current": 3752, "min": 0, "max": 4095, "robot_id": 22},
            63: {"name": "L Arm Rot", "current": 2173, "min": 0, "max": 4095, "robot_id": 23},
            64: {"name": "L Elbow", "current": 3645, "min": 0, "max": 4095, "robot_id": 24},
            65: {"name": "L Wrist Rot", "current": 2012, "min": 0, "max": 4095, "robot_id": 25},
            66: {"name": "L Wrist F/B", "current": 2016, "min": 0, "max": 4095, "robot_id": 26},
            67: {"name": "L Wrist L/R", "current": 2179, "min": 0, "max": 4095, "robot_id": 27},
            68: {"name": "L Gripper", "current": 10, "min": 10, "max": 4035, "robot_id": 31},
        }

        self.teleop_sliders = {}
        self.teleop_value_labels = {}
        self.teleop_connected = False
        self.teleop_ser = None
        self.teleop_ser_lock = threading.Lock()  # 시리얼 통신 락

        # 텔레옵 베이스라인 저장 파일 (V4 전용)
        self.teleop_baseline_file = os.path.join(_THIS_DIR, "teleop_baseline_v5.json")

        # 텔레옵 베이스라인 기본값 저장 (나중에 로드 시 사용)
        self.teleop_default_baseline = {
            51: 2964, 52: 92, 53: 1923, 54: 4095, 55: 2998, 56: 2338, 57: 2978, 58: 1,
            61: 1241, 62: 3752, 63: 2173, 64: 3645, 65: 2012, 66: 2016, 67: 2179, 68: 0,
        }

        # 기어 비율 정의 (텔레옵 -> 로봇)
        # 1:3 기어박스 모터: 텔레옵 1 움직이면 로봇 3 움직임
        # 1:1 모터: 텔레옵과 로봇 동일하게 움직임
        self.teleop_gear_ratio = {
            # 오른팔 - 1:3 기어박스
            51: 3, 52: 3, 53: 3, 54: 3,  # R Elbow 포함 전부 3배
            # 오른팔 - 1:1 (손목)
            55: 1, 56: 1, 57: 1,
            # 오른팔 - 그리퍼
            58: -1,
            # 왼팔 - 1:3 기어박스
            61: 3, 62: 3, 63: 3, 64: 3,  # L Arm Rot, L Elbow 포함 전부 3배
            # 왼팔 - 1:1 (손목)
            65: 1, 66: 1, 67: 1,
            # 왼팔 - 그리퍼
            68: -1,
        }

        # 텔레옵 슬라이더 표시 반전 (ROS Sim 방식, 4095 - pos)
        # L Elbow (64)는 방향이 반대이므로 슬라이더 표시를 반전
        self.teleop_display_inverted = [64]  # 반전할 텔레옵 서보 ID 목록

        # 그리퍼 설정: 텔레옵 750 변화 = 로봇 4095 변화
        # 왼쪽(68->31): 텔레옵 0->750 증가 -> 로봇 0->4095 증가
        # 오른쪽(58->41): 텔레옵 0->750 증가 -> 로봇 4095->0 감소
        self.gripper_config = {
            58: {"teleop_range": 750, "robot_range": 4095, "direction": -1},  # 오른손: 반대
            68: {"teleop_range": 750, "robot_range": 4095, "direction": 1},   # 왼손: 같은방향
        }

        # 로봇 기본자세 (V4 전용)
        self.robot_baseline_file = os.path.join(_THIS_DIR, "rx1_baseline_v5.json")
        self.robot_baseline = {}
        self.load_robot_baseline()

        # VR 텔레옵 전용 기본자세 (팔을 더 내린 자세)
        self.vr_baseline = {
            11: 2918, 12: 186, 13: 1984, 14: 933, 15: 2991, 16: 1931, 17: 2680,
            21: 1131, 22: 4003, 23: 1856, 24: 3280, 25: 2061, 26: 2139, 27: 1839,
            31: 10, 41: 10
        }

        # 현재 활성 baseline 추적 ('robot' or 'vr')
        self.active_baseline_type = 'robot'

        # 텔레옵->로봇 연동 활성화 여부
        self.teleop_to_robot_active = False
        self.teleop_sync_thread = None
        self._teleop_play_active = False  # 텔레옵 Play 모드 (비디오/저장 없이)

        # 로봇 실제 위치 캐시 (텔레옵 루프에서 읽어서 저장, 녹화에서 사용)
        self.cached_robot_positions = {}

        # 텔레옵 자동 종료 타이머 (분 단위, 0=무한)
        self.teleop_auto_stop_minutes = tk.StringVar(value="20")  # 기본 20분
        self.teleop_start_time = None  # 연동 시작 시간
        self.teleop_timer_id = None  # 타이머 ID

        # 저장된 베이스라인 로드
        self.load_teleop_baseline()

    def create_ui(self):
        """UI 생성 - 좌중우 분할"""
        # 창 크기 확장
        self.root.geometry("2496x1300")

        # ===== 시스템 모니터링 상태바 (최상단) =====
        status_bar = tk.Frame(self.root, bg='#1a1a2e', height=30)
        status_bar.pack(fill=tk.X, side=tk.TOP)
        status_bar.pack_propagate(False)

        # CPU 사용량
        tk.Label(status_bar, text="CPU:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(10, 2))
        self.cpu_label = tk.Label(status_bar, text="0%", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=5)
        self.cpu_label.pack(side=tk.LEFT)

        # RAM 사용량
        tk.Label(status_bar, text="RAM:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.ram_label = tk.Label(status_bar, text="0%", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=5)
        self.ram_label.pack(side=tk.LEFT)

        # GPU 사용량
        tk.Label(status_bar, text="GPU:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.gpu_label = tk.Label(status_bar, text="0%", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=5)
        self.gpu_label.pack(side=tk.LEFT)

        # GPU 메모리
        tk.Label(status_bar, text="VRAM:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.vram_label = tk.Label(status_bar, text="0MB", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=8)
        self.vram_label.pack(side=tk.LEFT)

        # CPU 온도
        tk.Label(status_bar, text="CPU온도:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.temp_cpu_label = tk.Label(status_bar, text="0°C", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=5)
        self.temp_cpu_label.pack(side=tk.LEFT)

        # GPU 온도
        tk.Label(status_bar, text="GPU온도:", bg='#1a1a2e', fg='white', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.temp_gpu_label = tk.Label(status_bar, text="0°C", bg='#1a1a2e', fg='#00ff00', font=('Noto Sans CJK KR', 12, 'bold'), width=5)
        self.temp_gpu_label.pack(side=tk.LEFT)

        # 서보 온도 (오른팔 최고)
        tk.Label(status_bar, text="R:", bg='#1a1a2e', fg='#87CEEB', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(15, 2))
        self.servo_temp_right_label = tk.Label(status_bar, text="--", bg='#1a1a2e', fg='gray', font=('Noto Sans CJK KR', 12, 'bold'), width=12)
        self.servo_temp_right_label.pack(side=tk.LEFT)

        # 서보 온도 (왼팔 최고)
        tk.Label(status_bar, text="L:", bg='#1a1a2e', fg='#FFA07A', font=('Noto Sans CJK KR', 12)).pack(side=tk.LEFT, padx=(10, 2))
        self.servo_temp_left_label = tk.Label(status_bar, text="--", bg='#1a1a2e', fg='gray', font=('Noto Sans CJK KR', 12, 'bold'), width=12)
        self.servo_temp_left_label.pack(side=tk.LEFT)

        # 경고 라벨
        self.warning_label = tk.Label(status_bar, text="", bg='#1a1a2e', fg='#ff4444', font=('Noto Sans CJK KR', 12, 'bold'))
        self.warning_label.pack(side=tk.RIGHT, padx=10)

        # 시스템 모니터링 스레드 시작
        threading.Thread(target=self._system_monitor_loop, daemon=True).start()

        # 메인 프레임
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 좌중우 분할
        # 왼쪽: 로봇 제어
        left_frame = ttk.LabelFrame(main_frame, text="RX-1 Robot Controller", padding=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # 가운데: 데이터 수집 (Pi0 파인튜닝용)
        center_frame = ttk.LabelFrame(main_frame, text="Data Collector (Pi0)", padding=10)
        center_frame.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=5)

        # 오른쪽: 바퀴 제어
        right_frame = ttk.LabelFrame(main_frame, text="Carter Wheel Control", padding=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False, padx=(5, 0))

        # 왼쪽 패널 구성
        self.create_control_panel(left_frame)
        self.create_slider_panel(left_frame)
        self.create_status_panel(left_frame)

        # 가운데 패널 구성 - 데이터 수집
        self.create_data_collection_panel(center_frame)

        # 오른쪽 패널 구성
        self.create_wheel_slider_panel(right_frame)

    def create_control_panel(self, parent):
        """제어 패널"""
        # ROS 연동 버튼 프레임 (상단)
        link_frame = ttk.Frame(parent)
        link_frame.pack(fill=tk.X, pady=(0, 5))

        # ROS 연결 버튼
        self.ros_connect_btn = ttk.Button(
            link_frame,
            text="ROS 연동",
            command=self.connect_ros,
            width=20
        )
        self.ros_connect_btn.pack(side=tk.LEFT, padx=5)

        self.ros_status = ttk.Label(
            link_frame,
            text="[ ] ROS 꺼짐",
            foreground="gray",
            font=("Noto Sans CJK KR", 12, "bold")
        )
        self.ros_status.pack(side=tk.LEFT, padx=10)

        # VR 텔레옵 버튼 (ROS 버튼 옆)
        ttk.Separator(link_frame, orient='vertical').pack(side=tk.LEFT, fill=tk.Y, padx=5)

        self.vr_teleop_btn = tk.Button(
            link_frame,
            text="VR 텔레옵",
            command=self.toggle_vr_teleop,
            width=10,
            bg='#6c5ce7',
            fg='white',
            font=("Noto Sans CJK KR", 10, "bold")
        )
        self.vr_teleop_btn.pack(side=tk.LEFT, padx=3)

        self.vr_teleop_status = ttk.Label(link_frame, text="[VR OFF]", foreground="gray")
        self.vr_teleop_status.pack(side=tk.LEFT, padx=3)

        # VR 팔 선택 버튼
        self.vr_arm_all_btn = tk.Button(
            link_frame, text="All", width=4,
            bg='#00b894', fg='white', font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self.set_vr_arm_select("all")
        )
        self.vr_arm_all_btn.pack(side=tk.LEFT, padx=1)

        self.vr_arm_left_btn = tk.Button(
            link_frame, text="L", width=3,
            bg='#636e72', fg='white', font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self.set_vr_arm_select("left")
        )
        self.vr_arm_left_btn.pack(side=tk.LEFT, padx=1)

        self.vr_arm_right_btn = tk.Button(
            link_frame, text="R", width=3,
            bg='#636e72', fg='white', font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self.set_vr_arm_select("right")
        )
        self.vr_arm_right_btn.pack(side=tk.LEFT, padx=1)

        # ========== 1행: 로봇 제어 ==========
        row1_frame = ttk.LabelFrame(parent, text="Robot 제어", padding=3)
        row1_frame.pack(fill=tk.X, pady=(0, 5))

        # 연결 버튼
        self.real_connect_btn = ttk.Button(
            row1_frame,
            text="로봇 연결",
            command=self.toggle_real_connection,
            width=12
        )
        self.real_connect_btn.pack(side=tk.LEFT, padx=3)

        # 연결 상태
        self.real_status_label = ttk.Label(
            row1_frame,
            text="[X] 연결안됨",
            foreground="red"
        )
        self.real_status_label.pack(side=tk.LEFT, padx=3)

        # 실시간 업데이트
        ttk.Checkbutton(
            row1_frame,
            text="실시간",
            variable=self.real_time_enabled
        ).pack(side=tk.LEFT, padx=3)

        # 기본 자세
        ttk.Button(
            row1_frame,
            text="Baseline",
            command=self.move_real_to_baseline,
            width=8
        ).pack(side=tk.LEFT, padx=3)

        # VR 기본자세 (VR 텔레옵용)
        ttk.Button(
            row1_frame,
            text="VR Pose",
            command=self.move_real_to_vr_baseline,
            width=8
        ).pack(side=tk.LEFT, padx=3)

        # 최소 부하 자세 (V3 기본자세)
        ttk.Button(
            row1_frame,
            text="최소부하",
            command=self.move_to_min_load_pose,
            width=10
        ).pack(side=tk.LEFT, padx=3)

        # 현재 자세 저장
        ttk.Button(
            row1_frame,
            text="자세저장",
            command=self.save_current_pose,
            width=10
        ).pack(side=tk.LEFT, padx=3)

        # ACC (가속도) 설정 프레임
        acc_frame = ttk.LabelFrame(row1_frame, text="ACC", padding=2)
        acc_frame.pack(side=tk.LEFT, padx=3)

        acc_entry = ttk.Entry(
            acc_frame,
            textvariable=self.real_acceleration_value,
            width=5,
            font=("Noto Sans CJK KR", 12, "bold")
        )
        acc_entry.pack(side=tk.LEFT, padx=2)

        ttk.Button(
            acc_frame,
            text="적용",
            command=self.apply_real_acceleration,
            width=5
        ).pack(side=tk.LEFT, padx=2)

        # Velocity (Sim 속도) 설정 프레임
        velocity_frame = ttk.LabelFrame(row1_frame, text="Sim Velocity", padding=2)
        velocity_frame.pack(side=tk.LEFT, padx=3)

        velocity_entry = ttk.Entry(
            velocity_frame,
            textvariable=self.sim_velocity_value,
            width=5,
            font=("Noto Sans CJK KR", 12, "bold")
        )
        velocity_entry.pack(side=tk.LEFT, padx=2)

        ttk.Label(velocity_frame, text="rad/s", font=("Noto Sans CJK KR", 10)).pack(side=tk.LEFT, padx=2)

        # 연동 모드 프레임
        sync_frame = ttk.LabelFrame(row1_frame, text="연동모드", padding=2)
        sync_frame.pack(side=tk.LEFT, padx=3)

        ttk.Radiobutton(sync_frame, text="독립", variable=self.sync_mode, value="독립",
                       command=self.change_sync_mode).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(sync_frame, text="Real->Sim", variable=self.sync_mode, value="Real->Sim",
                       command=self.change_sync_mode).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(sync_frame, text="Sim->Real", variable=self.sync_mode, value="Sim->Real",
                       command=self.change_sync_mode).pack(side=tk.LEFT, padx=2)

        # 네트워크 연결 버튼 (v8 GUI 연결)
        self.network_connect_btn = ttk.Button(
            sync_frame,
            text="NET 연결",
            command=self.toggle_network_connection,
            width=10
        )
        self.network_connect_btn.pack(side=tk.LEFT, padx=5)

        self.network_status_label = ttk.Label(sync_frame, text="[X]", foreground="red")
        self.network_status_label.pack(side=tk.LEFT)

        # 위치 읽기(Read-back) 버튼
        self.readback_btn = ttk.Button(
            sync_frame,
            text="위치읽기",
            command=self.toggle_readback,
            width=10
        )
        self.readback_btn.pack(side=tk.LEFT, padx=5)

        # ========== 2행: 텔레옵 제어 ==========
        row2_frame = ttk.LabelFrame(parent, text="Teleop 제어", padding=3)
        row2_frame.pack(fill=tk.X, pady=(0, 5))

        # 텔레옵 연결 버튼
        self.teleop_connect_btn = ttk.Button(
            row2_frame,
            text="텔레옵 연결",
            command=self.toggle_teleop_connection,
            width=12
        )
        self.teleop_connect_btn.pack(side=tk.LEFT, padx=3)

        # 텔레옵 연결 상태
        self.teleop_status_label = ttk.Label(
            row2_frame,
            text="[X] 연결안됨",
            foreground="red"
        )
        self.teleop_status_label.pack(side=tk.LEFT, padx=3)

        # 텔레옵 실시간 업데이트
        self.teleop_realtime_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            row2_frame,
            text="실시간",
            variable=self.teleop_realtime_enabled
        ).pack(side=tk.LEFT, padx=3)

        # 텔레옵 기본자세
        ttk.Button(
            row2_frame,
            text="기본자세",
            command=self.move_teleop_to_baseline,
            width=10
        ).pack(side=tk.LEFT, padx=3)

        # 텔레옵 기본자세 저장
        ttk.Button(
            row2_frame,
            text="자세저장",
            command=self.save_teleop_baseline,
            width=10
        ).pack(side=tk.LEFT, padx=3)

        # 텔레옵 현재위치 읽기
        ttk.Button(
            row2_frame,
            text="위치읽기",
            command=self.read_teleop_positions,
            width=10
        ).pack(side=tk.LEFT, padx=3)

        # 텔레옵 토크 ON/OFF
        self.teleop_torque_enabled = tk.BooleanVar(value=False)
        self.teleop_torque_btn = ttk.Button(
            row2_frame,
            text="토크 OFF",
            command=self.toggle_teleop_torque,
            width=10
        )
        self.teleop_torque_btn.pack(side=tk.LEFT, padx=3)

        # 텔레옵 ACC 설정
        teleop_acc_frame = ttk.LabelFrame(row2_frame, text="ACC", padding=2)
        teleop_acc_frame.pack(side=tk.LEFT, padx=3)

        self.teleop_acceleration_value = tk.IntVar(value=50)
        teleop_acc_entry = ttk.Entry(
            teleop_acc_frame,
            textvariable=self.teleop_acceleration_value,
            width=5,
            font=("Noto Sans CJK KR", 12, "bold")
        )
        teleop_acc_entry.pack(side=tk.LEFT, padx=2)

        ttk.Button(
            teleop_acc_frame,
            text="적용",
            command=self.apply_teleop_acceleration,
            width=5
        ).pack(side=tk.LEFT, padx=2)

        # 텔레옵->로봇 연동 버튼
        self.teleop_sync_btn = tk.Button(
            row2_frame,
            text="연동 OFF",
            command=self.toggle_teleop_to_robot_sync,
            width=12,
            bg="gray",
            fg="white",
            font=("Noto Sans CJK KR", 12, "bold")
        )
        self.teleop_sync_btn.pack(side=tk.LEFT, padx=3)

        # VR 스케일 (VR 텔레옵 버튼은 상단으로 이동됨)
        ttk.Separator(row2_frame, orient='vertical').pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(row2_frame, text="VR Scale:").pack(side=tk.LEFT, padx=(3, 1))
        vr_scale_spin = ttk.Spinbox(
            row2_frame,
            from_=100,
            to=1000,
            textvariable=self.vr_scale,
            width=5
        )
        vr_scale_spin.pack(side=tk.LEFT, padx=1)

        # 팔 선택 토글 (All, Right, Left)
        self.teleop_arm_select = tk.StringVar(value="All")
        ttk.Label(row2_frame, text="Arm:").pack(side=tk.LEFT, padx=(5, 2))

        arm_toggle_frame = ttk.Frame(row2_frame)
        arm_toggle_frame.pack(side=tk.LEFT, padx=2)

        self.arm_btn_all = tk.Button(
            arm_toggle_frame, text="All", width=4, bg="#4CAF50", fg="white",
            font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self._select_arm("All")
        )
        self.arm_btn_all.pack(side=tk.LEFT, padx=1)

        self.arm_btn_right = tk.Button(
            arm_toggle_frame, text="R", width=3, bg="gray", fg="white",
            font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self._select_arm("Right")
        )
        self.arm_btn_right.pack(side=tk.LEFT, padx=1)

        self.arm_btn_left = tk.Button(
            arm_toggle_frame, text="L", width=3, bg="gray", fg="white",
            font=("Noto Sans CJK KR", 9, "bold"),
            command=lambda: self._select_arm("Left")
        )
        self.arm_btn_left.pack(side=tk.LEFT, padx=1)

        # 연동 Hz 설정
        ttk.Label(row2_frame, text="Hz:").pack(side=tk.LEFT, padx=(10, 2))
        self.sync_hz_var = tk.StringVar(value="60")
        self.sync_hz_entry = ttk.Entry(row2_frame, textvariable=self.sync_hz_var, width=5)
        self.sync_hz_entry.pack(side=tk.LEFT, padx=2)

        # 텔레옵 자동 종료 시간 설정
        ttk.Label(row2_frame, text="자동종료:").pack(side=tk.LEFT, padx=(10, 2))
        self.teleop_auto_stop_combo = ttk.Combobox(
            row2_frame,
            textvariable=self.teleop_auto_stop_minutes,
            values=["10", "20", "30", "무한"],
            width=5,
            state="readonly"
        )
        self.teleop_auto_stop_combo.pack(side=tk.LEFT, padx=2)
        ttk.Label(row2_frame, text="분").pack(side=tk.LEFT)

        # 남은 시간 표시 레이블
        self.teleop_remaining_label = ttk.Label(row2_frame, text="", foreground="orange")
        self.teleop_remaining_label.pack(side=tk.LEFT, padx=(10, 2))

        # ========== 3행: IK 제어 ==========
        self.create_ik_panel(parent)

    def create_ik_panel(self, parent):
        """IK (Inverse Kinematics) 제어 패널"""
        ik_frame = ttk.LabelFrame(parent, text="IK Control (End Effector XYZ)", padding=3)
        ik_frame.pack(fill=tk.X, pady=(0, 5))

        # IK 활성화 체크박스와 버튼
        control_row = ttk.Frame(ik_frame)
        control_row.pack(fill=tk.X, pady=2)

        ttk.Checkbutton(
            control_row,
            text="IK 활성화",
            variable=self.ik_enabled,
            command=self.on_ik_toggle
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            control_row,
            text="현재위치 읽기",
            command=self.read_current_ik_position,
            width=14
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            control_row,
            text="IK 초기화",
            command=self.init_ik_solver,
            width=10
        ).pack(side=tk.LEFT, padx=5)

        # IK 상태 라벨
        self.ik_status_label = ttk.Label(control_row, text="[IK OFF]", foreground="gray")
        self.ik_status_label.pack(side=tk.LEFT, padx=10)

        # 양팔 슬라이더 컨테이너
        arms_frame = ttk.Frame(ik_frame)
        arms_frame.pack(fill=tk.X, pady=5)

        # 오른팔 XYZ
        right_frame = ttk.LabelFrame(arms_frame, text="Right Arm", padding=3)
        right_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self._create_ik_arm_sliders(right_frame, 'right')

        # 왼팔 XYZ
        left_frame = ttk.LabelFrame(arms_frame, text="Left Arm", padding=3)
        left_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self._create_ik_arm_sliders(left_frame, 'left')

    def _create_ik_arm_sliders(self, parent, arm):
        """팔별 IK 슬라이더 생성 (delta 기반, 0 = baseline)"""
        # 위치 슬라이더 (XYZ delta from baseline)
        pos_frame = ttk.Frame(parent)
        pos_frame.pack(fill=tk.X)

        # 슬라이더 범위: baseline에서 ±60cm
        for axis, (min_val, max_val, unit) in [
            ('x', (-0.60, 0.60, 'm')),
            ('y', (-0.60, 0.60, 'm')),
            ('z', (-0.60, 0.60, 'm'))
        ]:
            row = ttk.Frame(pos_frame)
            row.pack(fill=tk.X, pady=1)

            ttk.Label(row, text=f"{axis.upper()}:", width=3).pack(side=tk.LEFT)

            slider = tk.Scale(
                row,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=180,
                showvalue=0,
                command=lambda val, a=arm, ax=axis: self.on_ik_slider_change(a, ax, float(val))
            )
            slider.set(self.ik_values[arm][axis])
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

            # Show delta in cm (0 = baseline)
            value_label = ttk.Label(row, text="0.0cm", width=8)
            value_label.pack(side=tk.LEFT)

            self.ik_sliders[arm][axis] = slider
            self.ik_value_labels[arm][axis] = value_label

    def on_ik_toggle(self):
        """IK 활성화/비활성화 토글"""
        if self.ik_enabled.get():
            if self.ik_solver['right'] is None or self.ik_solver['left'] is None:
                self.init_ik_solver()
            if self.ik_solver['right'] and self.ik_solver['left']:
                self.ik_status_label.config(text="[IK ON]", foreground="green")
                self.read_current_ik_position()
            else:
                self.ik_enabled.set(False)
                self.ik_status_label.config(text="[IK Error]", foreground="red")
        else:
            self.ik_status_label.config(text="[IK OFF]", foreground="gray")

    def init_ik_solver(self):
        """IK Solver init + baseline XYZ 계산"""
        try:
            from rx1_ik_custom import RX1ArmIK, ik_debug_clear
            ik_debug_clear()

            # 활성 baseline 선택 (VR 또는 Robot)
            active_baseline = self.vr_baseline if self.active_baseline_type == 'vr' else self.robot_baseline
            baseline_name = "VR" if self.active_baseline_type == 'vr' else "Robot"

            # VIZ_BASELINE_OVERRIDE 사용 (URDF 체인과 매칭)
            # _compute_baseline_xyz에서 active_baseline 서보값으로 FK 계산 → 올바른 XYZ 획득

            # Create IK solver
            right_baseline = {sid: self.robot_baseline.get(sid, 2048)
                              for sid in [11, 12, 13, 14, 15, 16, 17]}
            self.ik_solver['right'] = RX1ArmIK('right', robot_baseline=right_baseline)

            # Left arm: use same custom IK (rx1_ik_custom)
            try:
                left_baseline = {sid: self.robot_baseline.get(sid, 2048)
                                 for sid in [21, 22, 23, 24, 25, 26, 27]}
                self.ik_solver['left'] = RX1ArmIK('left', robot_baseline=left_baseline)
            except Exception as e:
                self.log_real(f"[IK] Left arm custom IK failed: {e}")
                self.ik_solver['left'] = None

            # Baseline XYZ 계산 (활성 baseline 서보값으로 FK 실행)
            self._compute_baseline_xyz(active_baseline)
            self.log_real(f"[IK] Custom IK solver ready ({baseline_name} baseline)")

            self.log_real("[IK] Custom IK solver ready (viz-matching, debug: ik_debug.txt)")
            self.ik_status_label.config(text="[IK Ready]", foreground="blue")
        except Exception as e:
            import traceback
            self.log_real(f"[IK] Solver init failed: {e}")
            self.log_real(traceback.format_exc())
            self.ik_solver = {'right': None, 'left': None}

    def _compute_baseline_xyz(self, baseline=None):
        """baseline 서보값으로 baseline XYZ 위치 및 RPY 방향 계산

        Args:
            baseline: 사용할 baseline dict. None이면 self.robot_baseline 사용
        """
        from rx1_ik_custom import rotation_matrix_to_rpy
        import numpy as np

        if baseline is None:
            baseline = self.robot_baseline

        for arm in ['right', 'left']:
            solver = self.ik_solver.get(arm)
            if solver is None:
                continue

            # baseline 서보값 수집
            servo_values = {}
            for sid in solver.servo_ids:
                servo_values[sid] = baseline.get(sid, 2048)

            # FK로 baseline XYZ 및 RPY 계산
            try:
                # 위치
                pos = solver.get_end_effector_position(servo_values)
                self.ik_baseline_xyz[arm]['x'] = pos[0]
                self.ik_baseline_xyz[arm]['y'] = pos[1]
                self.ik_baseline_xyz[arm]['z'] = pos[2]

                # 방향 (4x4 변환 행렬에서 회전 추출)
                T = solver.get_gripper_transform(servo_values)
                R = T[:3, :3]
                roll, pitch, yaw = rotation_matrix_to_rpy(R)
                self.ik_baseline_rpy[arm]['roll'] = roll
                self.ik_baseline_rpy[arm]['pitch'] = pitch
                self.ik_baseline_rpy[arm]['yaw'] = yaw

                self.log_real(f"[IK] {arm} baseline: X={pos[0]*100:.1f}, Y={pos[1]*100:.1f}, Z={pos[2]*100:.1f} cm")
                self.log_real(f"[IK] {arm} baseline RPY: R={np.degrees(roll):.1f}, P={np.degrees(pitch):.1f}, Y={np.degrees(yaw):.1f} deg")
            except Exception as e:
                import traceback
                self.log_real(f"[IK] {arm} baseline FK failed: {e}")
                self.log_real(traceback.format_exc())

    def read_current_ik_position(self):
        """Reset IK sliders to 0 (baseline position)"""
        if self.ik_solver['right'] is None:
            self.init_ik_solver()
        if self.ik_solver['right'] is None:
            return

        self._ik_updating = True
        try:
            for arm in ['right', 'left']:
                solver = self.ik_solver[arm]
                if solver is None:
                    continue

                # Reset slider to 0 (baseline)
                for i, axis in enumerate(['x', 'y', 'z']):
                    self.ik_values[arm][axis] = 0.0
                    if axis in self.ik_sliders[arm]:
                        self.ik_sliders[arm][axis].set(0.0)
                    if axis in self.ik_value_labels[arm]:
                        self.ik_value_labels[arm][axis].config(text="0.0cm")
        finally:
            self._ik_updating = False

    def _init_ik_sliders_from_servo(self):
        """GUI 시작 시 IK 슬라이더를 현재 서보 FK 값으로 초기화"""
        try:
            self.init_ik_solver()
            self.read_current_ik_position()
        except Exception:
            pass  # IK 초기화 실패해도 GUI는 정상 작동

    def _get_current_joint_angles_rad(self, arm):
        """현재 서보 위치에서 라디안 관절 각도 추출 (IK용 - 서보 중앙 2048 = 0도)"""
        import numpy as np
        import math

        if arm == 'right':
            servo_ids = [11, 12, 13, 14, 15, 16, 17]
        else:
            servo_ids = [21, 22, 23, 24, 25, 26, 27]

        # 서보 특성 정의
        NO_GEARBOX_SERVOS = [15, 25]  # 360도 회전 (기어비 없음)
        INVERTED_SERVOS = [21, 22]  # 하드웨어 반전
        REASSEMBLED_SERVOS = [24]  # 하드웨어 재조립으로 방향 반전

        angles = []
        for sid in servo_ids:
            if sid in self.sim_active_joints:
                servo_val = self.sim_active_joints[sid]['current']

                # IK용: 서보 중앙(2048)을 0도로 사용
                relative_pos = servo_val - 2048

                # 기어비에 따른 각도 변환 (도 단위)
                if sid in NO_GEARBOX_SERVOS:
                    rel_angle = (relative_pos / 4095) * 360
                else:
                    rel_angle = (relative_pos / 4095) * 120

                # 하드웨어 반전 적용
                if sid in REASSEMBLED_SERVOS:
                    rel_angle = -rel_angle
                if sid in INVERTED_SERVOS:
                    rel_angle = -rel_angle

                # 라디안으로 변환
                rad = math.radians(rel_angle)
                angles.append(rad)
            else:
                angles.append(0)

        return np.array(angles)

    def on_ik_slider_change(self, arm, axis, value):
        """IK slider change callback (delta from baseline)"""
        # Skip if updating (prevent infinite loop)
        if self._ik_updating:
            return

        # Update value and label (show in cm)
        self.ik_values[arm][axis] = value
        if axis in self.ik_value_labels[arm]:
            self.ik_value_labels[arm][axis].config(text=f"{value*100:.1f}cm")

        # Only compute if IK is enabled
        if not self.ik_enabled.get():
            self.log_real(f"[IK] {arm} {axis}={value*100:.1f}cm (IK OFF)")
            return

        solver = self.ik_solver.get(arm)
        if not solver:
            self.log_real(f"[IK] {arm} solver not initialized")
            return

        self.log_real(f"[IK] {arm} {axis}={value*100:.1f}cm")

        # IK 계산 (스로틀링)
        if not self._ik_update_pending:
            self._ik_update_pending = True
            self.root.after(50, lambda: self._apply_ik(arm))

    def _check_servo_limits(self, servos):
        """서보 한계 체크 - 한계에 걸린 서보 목록 반환"""
        SERVO_MIN = 50
        SERVO_MAX = 4045
        limit_hit = []
        for sid, val in servos.items():
            if val <= SERVO_MIN:
                limit_hit.append(f"서보{sid}→하한")
            elif val >= SERVO_MAX:
                limit_hit.append(f"서보{sid}→상한")
        return limit_hit

    def _find_max_safe_position(self, arm, solver, prev_xyz, target_xyz, servo_values):
        """한계에 걸리지 않는 최대 위치를 이진 탐색으로 찾기"""
        # 이전 위치는 안전하다고 가정
        low, high = 0.0, 1.0
        best_ratio = 0.0
        best_servos = None

        for _ in range(8):  # 8번 반복 = 1/256 정밀도
            mid = (low + high) / 2
            test_xyz = [
                prev_xyz[0] + mid * (target_xyz[0] - prev_xyz[0]),
                prev_xyz[1] + mid * (target_xyz[1] - prev_xyz[1]),
                prev_xyz[2] + mid * (target_xyz[2] - prev_xyz[2])
            ]

            test_servos, success = solver.solve_for_position(test_xyz, servo_values)

            if success and not self._check_servo_limits(test_servos):
                # 이 위치는 안전 → 더 멀리 갈 수 있는지 확인
                best_ratio = mid
                best_servos = test_servos
                low = mid
            else:
                # 이 위치는 한계 → 더 가까이로
                high = mid

        if best_servos is None:
            return None, prev_xyz

        # 최대 안전 위치 계산
        safe_xyz = [
            prev_xyz[0] + best_ratio * (target_xyz[0] - prev_xyz[0]),
            prev_xyz[1] + best_ratio * (target_xyz[1] - prev_xyz[1]),
            prev_xyz[2] + best_ratio * (target_xyz[2] - prev_xyz[2])
        ]
        return best_servos, safe_xyz

    def _apply_ik(self, arm):
        """IK 계산 후 서보 슬라이더에 적용 (한계 체크 포함)"""
        self._ik_update_pending = False

        solver = self.ik_solver.get(arm)
        if not solver:
            return

        # baseline XYZ
        base_xyz = self.ik_baseline_xyz[arm]

        # Previous delta (for limit check)
        prev_delta = [
            self.ik_values[arm]['x'],
            self.ik_values[arm]['y'],
            self.ik_values[arm]['z']
        ]

        # Get delta from sliders (WORLD coordinates)
        delta_xyz = [
            self.ik_sliders[arm]['x'].get(),
            self.ik_sliders[arm]['y'].get(),
            self.ik_sliders[arm]['z'].get()
        ]

        # 활성 baseline 서보값 사용 (오차 누적 방지)
        active_baseline = self.vr_baseline if self.active_baseline_type == 'vr' else self.robot_baseline
        servo_values = {}
        for sid in solver.servo_ids:
            servo_values[sid] = active_baseline.get(sid, 2048)

        # WORLD IK: target = baseline position + delta
        target_xyz = [
            base_xyz['x'] + delta_xyz[0],
            base_xyz['y'] + delta_xyz[1],
            base_xyz['z'] + delta_xyz[2]
        ]
        new_servos, success = solver.solve_for_position(target_xyz, servo_values)

        if success:
            # Check servo limits
            limit_hit_servos = self._check_servo_limits(new_servos)

            if limit_hit_servos:
                # Limit hit - revert to previous delta
                limit_str = ", ".join(limit_hit_servos)
                self.log_real(f"[IK LIMIT] {arm}: {limit_str}")
                self._ik_updating = True
                try:
                    for i, axis in enumerate(['x', 'y', 'z']):
                        if axis in self.ik_sliders[arm]:
                            self.ik_sliders[arm][axis].set(prev_delta[i])
                        if axis in self.ik_value_labels[arm]:
                            self.ik_value_labels[arm][axis].config(text=f"{prev_delta[i]*100:.1f}cm")
                finally:
                    self._ik_updating = False
                self.ik_status_label.config(text=f"[Limit {arm}]", foreground="orange")
                return

            # Success - update servos
            self._ik_updating = True
            try:
                # Save delta values
                for i, axis in enumerate(['x', 'y', 'z']):
                    self.ik_values[arm][axis] = delta_xyz[i]

                # 손목 서보는 VR 회전으로 제어되므로 IK에서 제외
                wrist_servos = [15, 16, 17, 25, 26, 27]
                for sid, val in new_servos.items():
                    if sid in wrist_servos:
                        continue  # 손목 서보는 VR 회전으로 제어
                    if sid in self.real_sliders:
                        self.real_sliders[sid].set(val)
                    if sid in self.sim_active_joints:
                        self.sim_active_joints[sid]['current'] = val
                    if sid in self.real_active_joints:
                        self.real_active_joints[sid]['current'] = val
                    if sid in self.real_value_labels:
                        self.real_value_labels[sid].config(text=f"{val}")

                self.schedule_sim_update()
                self.ik_status_label.config(text=f"[IK OK - {arm}]", foreground="green")
            finally:
                self._ik_updating = False
        else:
            # IK failed - revert to previous delta
            self.log_real(f"[IK FAIL] {arm}: unreachable position")
            self._ik_updating = True
            try:
                for i, axis in enumerate(['x', 'y', 'z']):
                    if axis in self.ik_sliders[arm]:
                        self.ik_sliders[arm][axis].set(prev_delta[i])
                    if axis in self.ik_value_labels[arm]:
                        self.ik_value_labels[arm][axis].config(text=f"{prev_delta[i]*100:.1f}cm")
                self.ik_status_label.config(text=f"[IK Fail {arm}]", foreground="red")
            finally:
                self._ik_updating = False

    def create_slider_panel(self, parent):
        """슬라이더 패널 - 로봇과 텔레옵 나란히"""
        slider_frame = ttk.LabelFrame(parent, text="서보 컨트롤", padding=5)
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # 확대/축소 버튼
        zoom_frame = ttk.Frame(slider_frame)
        zoom_frame.pack(fill=tk.X, padx=5, pady=3)

        ttk.Label(zoom_frame, text="슬라이더 크기:", font=("Noto Sans CJK KR", 12)).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            zoom_frame,
            text="- 축소",
            width=8,
            command=self.zoom_out_sliders
        ).pack(side=tk.LEFT, padx=2)

        self.zoom_label = ttk.Label(zoom_frame, text="140%", font=("Noto Sans CJK KR", 12, "bold"))
        self.zoom_label.pack(side=tk.LEFT, padx=5)

        ttk.Button(
            zoom_frame,
            text="+ 확대",
            width=8,
            command=self.zoom_in_sliders
        ).pack(side=tk.LEFT, padx=2)

        # 스크롤 가능한 캔버스
        canvas = tk.Canvas(slider_frame, bg='#2c3e50', highlightthickness=0)
        scrollbar = ttk.Scrollbar(slider_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # 로봇 슬라이더와 텔레옵 슬라이더를 좌우로 배치할 컨테이너
        dual_slider_container = ttk.Frame(scrollable_frame)
        dual_slider_container.pack(fill=tk.BOTH, expand=True)

        # 왼쪽: 로봇 슬라이더
        robot_frame = ttk.LabelFrame(dual_slider_container, text="Robot", padding=3)
        robot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)

        # 오른쪽: 텔레옵 슬라이더
        teleop_frame = ttk.LabelFrame(dual_slider_container, text="Teleop", padding=3)
        teleop_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)

        # 로봇 슬라이더 생성 (28, 32 제외)
        for servo_id, joint_info in self.real_active_joints.items():
            if servo_id not in [28, 32]:
                self.create_joint_slider(robot_frame, servo_id, joint_info)

        # 텔레옵 슬라이더 생성
        for teleop_id, joint_info in self.teleop_active_joints.items():
            self.create_teleop_slider(teleop_frame, teleop_id, joint_info)



    def zoom_in_sliders(self):
        """슬라이더 10% 확대"""
        if self.slider_scale < 2.0:  # 최대 200%
            self.slider_scale += 0.1
            self.update_slider_zoom()

    def zoom_out_sliders(self):
        """슬라이더 10% 축소"""
        if self.slider_scale > 0.5:  # 최소 50%
            self.slider_scale -= 0.1
            self.update_slider_zoom()

    def update_slider_zoom(self):
        """슬라이더 크기 업데이트"""
        # 줌 레이블 업데이트
        percentage = int(self.slider_scale * 100)
        self.zoom_label.config(text=f"{percentage}%")

        new_length = int(440 * self.slider_scale)

        # 로봇 슬라이더 크기 업데이트
        for slider in self.real_sliders.values():
            slider.config(length=new_length)

        # 텔레옵 슬라이더 크기 업데이트
        for slider in self.teleop_sliders.values():
            slider.config(length=new_length)

    def on_real_slider_change(self, servo_id, value):
        """Real 슬라이더 값 변경 시 호출"""
        # 서보 26, 27 디버그 (파일로 저장)
        if servo_id in [26, 27]:
            from datetime import datetime
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with open("servo_26_27_debug.txt", "a") as f:
                f.write(f"[{ts}] [SLIDER] 서보 {servo_id}: value={value}, real_link_to_sim={self.real_link_to_sim}\n")

        # VR baseline 설정 중이면 완전히 스킵 (IK 계산 방지)
        if self._ik_updating:
            # 값만 업데이트하고 다른 처리는 스킵
            self.real_active_joints[servo_id]['current'] = value
            if servo_id in self.real_value_labels:
                self.real_value_labels[servo_id].config(text=f"{value}")
            if servo_id in self.sim_active_joints:
                self.sim_active_joints[servo_id]['current'] = value
            return

        # 현재 값 업데이트
        self.real_active_joints[servo_id]['current'] = value

        # 값 라벨 업데이트
        if servo_id in self.real_value_labels:
            self.real_value_labels[servo_id].config(text=f"{value}")

        # 베이스라인 로드 중이거나 Sim->Real 업데이트 중이면 모터 전송 건너뛰기
        if not (self.loading_baseline or self.updating_from_sim):
            # 실시간 업데이트가 켜져 있고 연결된 경우 하드웨어로 전송
            if self.real_time_enabled.get() and self.real_connected:
                # STS3215 프로토콜로 실제 모터에 명령 전송
                try:
                    # 서보 26, 27 디버그 (하드웨어 전송)
                    if servo_id in [26, 27]:
                        from datetime import datetime
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        with open("servo_26_27_debug.txt", "a") as f:
                            f.write(f"[{ts}] [HW SEND] 서보 {servo_id}: value={value}\n")
                    self.send_real_servo_command(servo_id, value)
                except Exception as e:
                    # 서보 26, 27 에러 로그
                    if servo_id in [26, 27]:
                        from datetime import datetime
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        with open("servo_26_27_debug.txt", "a") as f:
                            f.write(f"[{ts}] [HW ERROR] 서보 {servo_id}: {e}\n")
            else:
                # 서보 26, 27 디버그 (실시간 비활성화 또는 연결 안됨)
                if servo_id in [26, 27]:
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open("servo_26_27_debug.txt", "a") as f:
                        f.write(f"[{ts}] [HW DISABLED] 서보 {servo_id}: real_time={self.real_time_enabled.get()}, connected={self.real_connected}\n")
        else:
            # 서보 26, 27 디버그 (베이스라인/심 업데이트 중)
            if servo_id in [26, 27]:
                from datetime import datetime
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                with open("servo_26_27_debug.txt", "a") as f:
                    f.write(f"[{ts}] [HW SKIP] 서보 {servo_id}: loading_baseline={self.loading_baseline}, updating_from_sim={self.updating_from_sim}\n")

        # Real->Sim 모드가 아닐 때만 슬라이더 값으로 Sim 업데이트
        # Real->Sim 모드에서는 실제 모터 위치만 Sim에 반영 (update_real_positions_to_sim에서 처리)
        if not self.real_link_to_sim:
            # 독립 모드나 Sim->Real 모드에서는 슬라이더 값으로 Sim 동기화 가능
            if servo_id in self.sim_active_joints:
                self.sim_active_joints[servo_id]['current'] = value
                # 서보 26, 27 디버그 (파일로 저장)
                if servo_id in [26, 27]:
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open("servo_26_27_debug.txt", "a") as f:
                        f.write(f"[{ts}] [SIM UPDATE] sim_active_joints[{servo_id}] = {value}\n")
                # Sim 값이 변경되었으니 RViz에 반영 (스로틀링)
                self.schedule_sim_update()
        else:
            # 서보 26, 27 디버그 - real_link_to_sim 모드 (파일로 저장)
            if servo_id in [26, 27]:
                from datetime import datetime
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                with open("servo_26_27_debug.txt", "a") as f:
                    f.write(f"[{ts}] [SKIP] 서보 {servo_id}: real_link_to_sim=True, sim 업데이트 스킵됨\n")

        # IK 슬라이더 업데이트 (팔 관절 변경 시 FK로 end effector 위치 계산)
        # IK->서보 업데이트 중에는 무한 루프 방지를 위해 스킵
        # 오류 발생 시 무시 (IK 기능 미완성)
        try:
            if self.ik_solver and not self._updating_from_ik:
                if servo_id in [11, 12, 13, 14, 15, 16, 17]:
                    self._update_ik_sliders_from_fk('right')
                elif servo_id in [21, 22, 23, 24, 25, 26, 27]:
                    self._update_ik_sliders_from_fk('left')
        except Exception:
            pass

    def _update_ik_sliders_from_fk(self, arm):
        """Update IK sliders from current servo (show delta from baseline)"""
        solver = self.ik_solver.get(arm)
        if not solver:
            return

        try:
            # Current servo values
            servo_values = {}
            for sid in solver.servo_ids:
                if sid in self.sim_active_joints:
                    servo_values[sid] = self.sim_active_joints[sid]['current']
                else:
                    servo_values[sid] = 2048

            # FK to get current XYZ
            pos = solver.get_end_effector_position(servo_values)

            # Compute delta from baseline
            base_xyz = self.ik_baseline_xyz[arm]
            delta = [
                pos[0] - base_xyz['x'],
                pos[1] - base_xyz['y'],
                pos[2] - base_xyz['z']
            ]

            # Update IK sliders with delta
            # 외부에서 이미 설정된 _ik_updating 플래그 보존
            was_updating = self._ik_updating
            self._ik_updating = True
            try:
                for i, axis in enumerate(['x', 'y', 'z']):
                    self.ik_values[arm][axis] = delta[i]
                    if axis in self.ik_sliders[arm]:
                        self.ik_sliders[arm][axis].set(delta[i])
                    if axis in self.ik_value_labels[arm]:
                        self.ik_value_labels[arm][axis].config(text=f"{delta[i]*100:.1f}cm")
            finally:
                # 외부에서 설정된 상태 복원
                self._ik_updating = was_updating
        except Exception:
            pass  # 에러 시에도 플래그 변경하지 않음

    def create_joint_slider(self, parent, servo_id, joint_info):
        """개별 슬라이더 생성"""
        joint_frame = ttk.LabelFrame(parent, text=f"서보 {servo_id}", padding=3)
        joint_frame.pack(fill=tk.X, pady=3)

        ttk.Label(joint_frame, text=joint_info["name"], font=("Noto Sans CJK KR", 8, "bold")).pack()

        # 값 표시 + Sim 반전 버튼
        value_frame = ttk.Frame(joint_frame)
        value_frame.pack(fill=tk.X, pady=2)

        ttk.Label(value_frame, text="값:").pack(side=tk.LEFT)
        self.real_value_labels[servo_id] = ttk.Label(
            value_frame,
            text=f"{joint_info['current']}",
            font=("Noto Sans CJK KR", 12, "bold"),
            foreground="blue"
        )
        self.real_value_labels[servo_id].pack(side=tk.LEFT, padx=5)

        # Sim 반전 버튼
        self.sim_inverted[servo_id] = False  # 초기 상태
        invert_btn = ttk.Button(
            value_frame,
            text="[ ] Sim반전",
            command=lambda sid=servo_id: self.toggle_sim_invert(sid),
            width=10
        )
        invert_btn.pack(side=tk.RIGHT, padx=5)
        self.invert_buttons[servo_id] = invert_btn

        # 슬라이더 프레임 (방향 라벨 포함)
        slider_frame = ttk.Frame(joint_frame)
        slider_frame.pack(fill=tk.X, pady=2)

        # 방향 라벨 결정
        name = joint_info["name"]
        if "F/B" in name:
            left_label, right_label = "앞", "뒤"
            # 서보 26은 방향이 반대
            if servo_id == 26:
                left_label, right_label = "뒤", "앞"
        elif "L/R" in name:
            left_label, right_label = "좌", "우"
        elif "U/D" in name:
            left_label, right_label = "위", "아래"
        elif "Rot" in name:
            left_label, right_label = "좌", "우"
        elif "Gripper" in name:
            left_label, right_label = "열기", "닫기"
        else:
            left_label, right_label = "<-", "->"

        # 왼쪽 라벨
        ttk.Label(
            slider_frame,
            text=f"[{left_label}]",
            font=("Noto Sans CJK KR", 10),
            foreground="darkblue"
        ).pack(side=tk.LEFT, padx=2)

        # 슬라이더
        slider = tk.Scale(
            slider_frame,
            from_=joint_info['min'],
            to=joint_info['max'],
            orient=tk.HORIZONTAL,
            length=int(440 * self.slider_scale),
            resolution=1,
            command=lambda val, sid=servo_id: self.on_real_slider_change(sid, int(val)),
            showvalue=0
        )
        slider.set(joint_info['current'])
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.real_sliders[servo_id] = slider

        # 오른쪽 라벨
        ttk.Label(
            slider_frame,
            text=f"[{right_label}]",
            font=("Noto Sans CJK KR", 10),
            foreground="darkblue"
        ).pack(side=tk.LEFT, padx=2)

        # 범위 표시
        ttk.Label(
            joint_frame,
            text=f"{joint_info['min']} ~ {joint_info['max']}",
            font=("Noto Sans CJK KR", 12),
            foreground="gray"
        ).pack()

    def create_teleop_slider(self, parent, teleop_id, joint_info):
        """텔레오퍼레이션 슬라이더 생성"""
        joint_frame = ttk.LabelFrame(parent, text=f"텔레옵 {teleop_id} -> 로봇 {joint_info['robot_id']}", padding=3)
        joint_frame.pack(fill=tk.X, pady=3)

        ttk.Label(joint_frame, text=joint_info["name"], font=("Noto Sans CJK KR", 8, "bold")).pack()

        # 값 표시
        value_frame = ttk.Frame(joint_frame)
        value_frame.pack(fill=tk.X, pady=2)

        ttk.Label(value_frame, text="값:").pack(side=tk.LEFT)
        self.teleop_value_labels[teleop_id] = ttk.Label(
            value_frame,
            text=f"{joint_info['current']}",
            font=("Noto Sans CJK KR", 12, "bold"),
            foreground="orange"
        )
        self.teleop_value_labels[teleop_id].pack(side=tk.LEFT, padx=5)

        # 슬라이더 프레임
        slider_frame = ttk.Frame(joint_frame)
        slider_frame.pack(fill=tk.X, pady=2)

        # 방향 라벨 결정
        name = joint_info["name"]
        if "F/B" in name:
            left_label, right_label = "앞", "뒤"
        elif "L/R" in name:
            left_label, right_label = "좌", "우"
        elif "U/D" in name:
            left_label, right_label = "위", "아래"
        elif "Rot" in name:
            left_label, right_label = "좌", "우"
        elif "Gripper" in name:
            left_label, right_label = "열기", "닫기"
        else:
            left_label, right_label = "<-", "->"

        # 왼쪽 라벨
        ttk.Label(
            slider_frame,
            text=f"[{left_label}]",
            font=("Noto Sans CJK KR", 10),
            foreground="darkorange"
        ).pack(side=tk.LEFT, padx=2)

        # 슬라이더 (텔레옵용)
        slider = tk.Scale(
            slider_frame,
            from_=joint_info['min'],
            to=joint_info['max'],
            orient=tk.HORIZONTAL,
            length=int(440 * self.slider_scale),
            resolution=1,
            command=lambda val, tid=teleop_id: self.on_teleop_slider_change(tid, int(val)),
            showvalue=0,
            troughcolor='#3d5a80',
            activebackground='orange'
        )
        slider.set(joint_info['current'])
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.teleop_sliders[teleop_id] = slider

        # 오른쪽 라벨
        ttk.Label(
            slider_frame,
            text=f"[{right_label}]",
            font=("Noto Sans CJK KR", 10),
            foreground="darkorange"
        ).pack(side=tk.LEFT, padx=2)

        # 범위 표시
        ttk.Label(
            joint_frame,
            text=f"{joint_info['min']} ~ {joint_info['max']}",
            font=("Noto Sans CJK KR", 12),
            foreground="gray"
        ).pack()

    def on_teleop_slider_change(self, teleop_id, value):
        """텔레옵 슬라이더 값 변경 시 호출"""
        # 슬라이더 값(표시값)을 실제 위치로 변환
        raw_position = self.teleop_raw_position(teleop_id, value)

        # 현재 값 업데이트 (실제 위치 저장)
        self.teleop_active_joints[teleop_id]['current'] = raw_position

        # 값 라벨 업데이트 (표시값)
        if teleop_id in self.teleop_value_labels:
            self.teleop_value_labels[teleop_id].config(text=f"{value}")

        # 연동 중일 때: 슬라이더로 로봇 + 텔레옵 제어
        if self.teleop_to_robot_active:
            # 텔레옵 위치를 로봇 위치로 변환하여 전송
            robot_target = self.convert_teleop_to_robot(teleop_id, raw_position)
            if robot_target is not None:
                robot_id = self.teleop_to_robot_mapping.get(teleop_id)
                if robot_id:
                    # 로봇에 위치 명령 전송
                    self.sync_write_positions({robot_id: robot_target})
                    # 로봇 슬라이더도 업데이트
                    if robot_id in self.real_sliders:
                        self.real_sliders[robot_id].set(robot_target)

            # 텔레옵 토크 ON이면 텔레옵 장비도 따라가게
            if self.teleop_torque_enabled.get() and self.teleop_connected and self.teleop_ser:
                try:
                    self.send_teleop_servo_command(teleop_id, raw_position)
                except Exception as e:
                    pass
            return

        # 텔레옵 연결 시 해당 서보로 명령 전송 (실제 위치)
        if self.teleop_connected and self.teleop_ser:
            try:
                self.send_teleop_servo_command(teleop_id, raw_position)
            except Exception as e:
                pass

    def send_teleop_servo_command(self, teleop_id, position):
        """텔레옵 서보에 명령 전송 (스레드 안전)"""
        if not self.teleop_ser:
            return

        # 락 획득 시도 (최대 100ms 대기)
        if not self.teleop_ser_lock.acquire(timeout=0.1):
            return

        try:
            # STS3215/SCS 프로토콜 위치 명령
            pos_low = position & 0xFF
            pos_high = (position >> 8) & 0xFF

            # Write Position 패킷
            packet = bytearray([0xFF, 0xFF, teleop_id, 0x05, 0x03, 0x2A, pos_low, pos_high])
            checksum = (~(teleop_id + 0x05 + 0x03 + 0x2A + pos_low + pos_high)) & 0xFF
            packet.append(checksum)

            self.teleop_ser.write(packet)
            self.teleop_ser.flush()
            time.sleep(0.002)  # 2ms 대기
        except Exception as e:
            self.log_real(f"[텔레옵] 명령 전송 실패: {e}")
        finally:
            self.teleop_ser_lock.release()

    # ===================================================================
    # 텔레옵 제어 함수들
    # ===================================================================

    def toggle_teleop_connection(self):
        """텔레옵 연결/해제 토글"""
        if self.teleop_connected:
            self.disconnect_teleop()
        else:
            self.connect_teleop()

    def find_teleop_port(self):
        """텔레옵 서보가 연결된 USB 포트 자동 감지"""
        import glob

        available_ports = glob.glob('/dev/ttyUSB*')
        self.log_real(f"[텔레옵 스캔] USB 포트 검색 중... {available_ports}")

        # 텔레옵 서보 ID (51-68)
        teleop_test_ids = [51, 52, 61, 62]

        for port in available_ports:
            try:
                test_ser = serial.Serial(port, 1000000, timeout=0.02)
                time.sleep(0.05)

                servo_found = False
                for servo_id in teleop_test_ids:
                    packet = bytearray([0xFF, 0xFF, servo_id, 0x02, 0x01])
                    checksum = (~(servo_id + 0x02 + 0x01)) & 0xFF
                    packet.append(checksum)

                    test_ser.write(packet)
                    test_ser.flush()
                    time.sleep(0.01)

                    response = test_ser.read(20)
                    if len(response) >= 6:
                        servo_found = True
                        self.log_real(f"[텔레옵 스캔] {port}: 텔레옵 서보 ID {servo_id} 발견!")
                        break

                test_ser.close()

                if servo_found:
                    return port

            except Exception as e:
                self.log_real(f"[텔레옵 스캔] {port} 오류: {e}")

        return None

    def connect_teleop(self):
        """텔레옵 연결 - 자동 포트 감지"""
        try:
            # 자동으로 텔레옵 포트 찾기
            found_port = self.find_teleop_port()

            if not found_port:
                self.log_real("[텔레옵] 자동 감지 실패, /dev/ttyUSB0 시도...")
                found_port = '/dev/ttyUSB0'

            self.teleop_ser = serial.Serial(found_port, 1000000, timeout=0.02)
            self.teleop_ser.reset_input_buffer()
            self.teleop_ser.reset_output_buffer()
            self.teleop_connected = True

            self.teleop_connect_btn.config(text="텔레옵 해제")
            self.teleop_status_label.config(text="[O] 연결됨", foreground="green")

            self.log_real(f"[텔레옵] 연결 성공 ({found_port})")

            # 텔레옵 ACC 기본값 10 설정
            self.set_teleop_acc(10)

        except Exception as e:
            messagebox.showerror("텔레옵 연결 오류", f"연결 실패:\n{e}")
            self.log_real(f"[텔레옵] 연결 실패: {e}")

    def set_teleop_acc(self, acc_value):
        """텔레옵 서보 ACC 설정 (스레드 안전)"""
        if not self.teleop_connected or not self.teleop_ser:
            return

        self.log_real(f"[텔레옵] ACC 설정 중: {acc_value}")

        if not self.teleop_ser_lock.acquire(timeout=0.5):
            self.log_real("[텔레옵] 락 획득 실패")
            return

        try:
            for teleop_id in self.teleop_active_joints.keys():
                try:
                    # ACC 설정 패킷 (주소 0x29)
                    packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x03, 0x29, acc_value])
                    checksum = (~(teleop_id + 0x04 + 0x03 + 0x29 + acc_value)) & 0xFF
                    packet.append(checksum)
                    self.teleop_ser.write(packet)
                    self.teleop_ser.flush()
                    time.sleep(0.002)
                except:
                    pass
            time.sleep(0.05)
            self.teleop_ser.reset_input_buffer()
        finally:
            self.teleop_ser_lock.release()

        self.log_real(f"[텔레옵] ACC 설정 완료: {acc_value}")

    def disconnect_teleop(self):
        """텔레옵 연결 해제"""
        if self.teleop_ser:
            self.teleop_ser.close()
            self.teleop_ser = None

        self.teleop_connected = False
        self.teleop_connect_btn.config(text="텔레옵 연결")
        self.teleop_status_label.config(text="[X] 연결안됨", foreground="red")
        self.log_real("[텔레옵] 연결 해제됨")

    def move_teleop_to_baseline(self):
        """텔레옵 기본자세로 이동 (토크 ON 후 이동, 스레드 안전)"""
        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        # 연동 중이면 잠시 중지
        was_syncing = self.teleop_to_robot_active
        if was_syncing:
            self.log_real("[텔레옵] 기본자세 이동을 위해 연동 일시 중지...")
            self.teleop_to_robot_active = False
            time.sleep(0.2)  # 연동 루프가 종료될 시간

        # 1. 토크 ON (락 보호)
        self.log_real("[텔레옵] 토크 ON 설정 중...")
        if self.teleop_ser_lock.acquire(timeout=0.5):
            try:
                # 버퍼 클리어
                self.teleop_ser.reset_input_buffer()
                self.teleop_ser.reset_output_buffer()
                time.sleep(0.05)

                for teleop_id in self.teleop_active_joints.keys():
                    try:
                        packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x03, 0x28, 0x01])
                        checksum = (~(teleop_id + 0x04 + 0x03 + 0x28 + 0x01)) & 0xFF
                        packet.append(checksum)
                        self.teleop_ser.write(packet)
                        self.teleop_ser.flush()
                        time.sleep(0.005)  # 5ms 대기
                    except:
                        pass
                time.sleep(0.1)
                self.teleop_ser.reset_input_buffer()
            finally:
                self.teleop_ser_lock.release()
        else:
            self.log_real("[텔레옵] 락 획득 실패, 재시도 필요")
            return

        self.teleop_torque_enabled.set(True)
        self.teleop_torque_btn.config(text="토크 ON")
        self.log_real("[텔레옵] 토크 ON 완료")

        # 2. 저장된 베이스라인 로드
        self.load_teleop_baseline()

        # 3. 기본자세로 이동
        self.log_real("[텔레옵] 기본자세로 이동 중...")
        for teleop_id, joint_info in self.teleop_active_joints.items():
            baseline_pos = joint_info['current']  # 로드된 기본자세
            self.send_teleop_servo_command(teleop_id, baseline_pos)

            # 슬라이더 업데이트 (반전 적용)
            display_pos = self.teleop_display_position(teleop_id, baseline_pos)
            if teleop_id in self.teleop_sliders:
                self.teleop_sliders[teleop_id].set(display_pos)
            if teleop_id in self.teleop_value_labels:
                self.teleop_value_labels[teleop_id].config(text=f"{display_pos}")

            time.sleep(0.02)  # 20ms 대기

        self.log_real("[텔레옵] 기본자세 이동 완료")

        # 연동이 실행 중이었으면 다시 시작
        if was_syncing:
            self.log_real("[텔레옵] 연동 재시작...")
            self.teleop_to_robot_active = True
            sync_thread = threading.Thread(target=self.teleop_to_robot_sync_loop, daemon=True)
            sync_thread.start()

    def read_teleop_positions(self):
        """텔레옵 현재 위치 읽기 (스레드 안전)"""
        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        self.log_real("[텔레옵] 현재 위치 읽기 중...")

        if not self.teleop_ser_lock.acquire(timeout=0.5):
            self.log_real("[텔레옵] 락 획득 실패")
            return

        try:
            for teleop_id in self.teleop_active_joints.keys():
                try:
                    # Read Position 패킷 (주소 0x38, 길이 2)
                    packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x02, 0x38, 0x02])
                    checksum = (~(teleop_id + 0x04 + 0x02 + 0x38 + 0x02)) & 0xFF
                    packet.append(checksum)

                    self.teleop_ser.write(packet)
                    self.teleop_ser.flush()
                    time.sleep(0.01)

                    response = self.teleop_ser.read(20)
                    if len(response) >= 8:
                        pos_low = response[5]
                        pos_high = response[6]
                        position = pos_low + (pos_high << 8)

                        self.teleop_active_joints[teleop_id]['current'] = position

                        # 슬라이더 표시용 위치 (반전 적용)
                        display_pos = self.teleop_display_position(teleop_id, position)
                        if teleop_id in self.teleop_sliders:
                            self.teleop_sliders[teleop_id].set(display_pos)
                        if teleop_id in self.teleop_value_labels:
                            self.teleop_value_labels[teleop_id].config(text=f"{display_pos}")

                except Exception as e:
                    self.log_real(f"[텔레옵] ID {teleop_id} 읽기 실패: {e}")
        finally:
            self.teleop_ser_lock.release()

        self.log_real("[텔레옵] 위치 읽기 완료")

    def toggle_teleop_torque(self):
        """텔레옵 토크 ON/OFF 토글 (스레드 안전)"""
        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        self.teleop_torque_enabled.set(not self.teleop_torque_enabled.get())
        torque_on = self.teleop_torque_enabled.get()

        if not self.teleop_ser_lock.acquire(timeout=0.5):
            self.log_real("[텔레옵] 락 획득 실패")
            return

        try:
            for teleop_id in self.teleop_active_joints.keys():
                try:
                    # Torque Enable (주소 0x28, 값 0 또는 1)
                    torque_val = 1 if torque_on else 0
                    packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x03, 0x28, torque_val])
                    checksum = (~(teleop_id + 0x04 + 0x03 + 0x28 + torque_val)) & 0xFF
                    packet.append(checksum)

                    self.teleop_ser.write(packet)
                    self.teleop_ser.flush()
                    time.sleep(0.005)

                except Exception as e:
                    self.log_real(f"[텔레옵] ID {teleop_id} 토크 설정 실패: {e}")
        finally:
            self.teleop_ser_lock.release()

        if torque_on:
            self.teleop_torque_btn.config(text="토크 ON")
            self.log_real("[텔레옵] 토크 ON - 서보 고정됨")
        else:
            self.teleop_torque_btn.config(text="토크 OFF")
            self.log_real("[텔레옵] 토크 OFF - 서보 자유 움직임")

    def apply_teleop_acceleration(self):
        """텔레옵 ACC 적용 (스레드 안전)"""
        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        acc_value = self.teleop_acceleration_value.get()
        self.log_real(f"[텔레옵] ACC {acc_value} 적용 중...")

        if not self.teleop_ser_lock.acquire(timeout=0.5):
            self.log_real("[텔레옵] 락 획득 실패")
            return

        try:
            for teleop_id in self.teleop_active_joints.keys():
                try:
                    # ACC 설정 (주소 0x29)
                    packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x03, 0x29, acc_value])
                    checksum = (~(teleop_id + 0x04 + 0x03 + 0x29 + acc_value)) & 0xFF
                    packet.append(checksum)

                    self.teleop_ser.write(packet)
                    self.teleop_ser.flush()
                    time.sleep(0.005)

                except Exception as e:
                    self.log_real(f"[텔레옵] ID {teleop_id} ACC 설정 실패: {e}")
        finally:
            self.teleop_ser_lock.release()

        self.log_real(f"[텔레옵] ACC {acc_value} 적용 완료")

    def save_teleop_baseline(self):
        """텔레옵 현재 위치를 기본자세로 저장"""
        try:
            # 현재 슬라이더 값들을 저장
            baseline_data = {}
            for teleop_id, joint_info in self.teleop_active_joints.items():
                baseline_data[str(teleop_id)] = joint_info['current']

            # JSON 파일로 저장
            import json
            with open(self.teleop_baseline_file, 'w') as f:
                json.dump(baseline_data, f, indent=2)

            self.log_real(f"[텔레옵] 기본자세 저장 완료: {self.teleop_baseline_file}")
            messagebox.showinfo("저장 완료", "텔레옵 기본자세가 저장되었습니다.")

        except Exception as e:
            self.log_real(f"[텔레옵] 기본자세 저장 실패: {e}")
            messagebox.showerror("저장 실패", f"기본자세 저장 중 오류:\n{e}")

    def load_teleop_baseline(self):
        """저장된 텔레옵 기본자세 로드"""
        import json
        import os

        # 별도의 기본자세 딕셔너리 (연동 계산용)
        self.teleop_baseline = dict(self.teleop_default_baseline)

        try:
            if os.path.exists(self.teleop_baseline_file):
                with open(self.teleop_baseline_file, 'r') as f:
                    baseline_data = json.load(f)

                # 로드된 값으로 업데이트 (기본자세 딕셔너리와 active_joints 둘 다)
                for teleop_id_str, position in baseline_data.items():
                    teleop_id = int(teleop_id_str)
                    if teleop_id in self.teleop_active_joints:
                        self.teleop_active_joints[teleop_id]['current'] = position
                        self.teleop_baseline[teleop_id] = position

                print(f"[텔레옵] 기본자세 로드 완료: {self.teleop_baseline_file}")
                print(f"[텔레옵] 기본자세: {self.teleop_baseline}")
            else:
                print(f"[텔레옵] 기본자세 파일 없음, 기본값 사용")

        except Exception as e:
            print(f"[텔레옵] 기본자세 로드 실패: {e}, 기본값 사용")

    def load_robot_baseline(self):
        """저장된 로봇 기본자세 로드 (rx1_baseline.json)"""
        import json
        import os

        try:
            if os.path.exists(self.robot_baseline_file):
                with open(self.robot_baseline_file, 'r') as f:
                    data = json.load(f)

                # servos 딕셔너리에서 position 값 추출
                if 'servos' in data:
                    for servo_id_str, servo_data in data['servos'].items():
                        servo_id = int(servo_id_str)
                        self.robot_baseline[servo_id] = servo_data.get('position', 2048)

                print(f"[로봇] 기본자세 로드 완료: {self.robot_baseline_file}")
                print(f"[로봇] 기본자세: {self.robot_baseline}")
            else:
                print(f"[로봇] 기본자세 파일 없음, 기본값 사용")
                # 기본값 설정
                self.robot_baseline = {
                    11: 2964, 12: 92, 13: 1923, 14: 4095, 15: 2995, 16: 2006, 17: 2980,
                    21: 1194, 22: 3906, 23: 2006, 24: 0, 25: 2112, 26: 2112, 27: 1854,
                    41: 0, 31: 0, 32: 0, 28: 0,
                }

        except Exception as e:
            print(f"[로봇] 기본자세 로드 실패: {e}, 기본값 사용")
            self.robot_baseline = {
                11: 2964, 12: 92, 13: 1923, 14: 4095, 15: 2995, 16: 2006, 17: 2980,
                21: 1194, 22: 3906, 23: 2006, 24: 0, 25: 2112, 26: 2112, 27: 1854,
                41: 0, 31: 0, 32: 0, 28: 0,
            }

        # 그리퍼 서보 10~4035 클램핑
        for gid in [41, 31, 32, 28]:
            if gid in self.robot_baseline:
                self.robot_baseline[gid] = max(10, min(4035, self.robot_baseline[gid]))

    # ===================================================================
    # 텔레옵 -> 로봇 연동 기능
    # ===================================================================

    def get_teleop_baseline(self, teleop_id):
        """텔레옵 기본자세 값 반환 (저장된 baseline 사용)"""
        return self.teleop_baseline.get(teleop_id, 2048)

    def get_robot_baseline(self, robot_id):
        """로봇 기본자세 값 반환"""
        return self.robot_baseline.get(robot_id, 2048)

    def calculate_teleop_effective_range(self, teleop_id):
        """
        텔레옵의 유효 범위 계산 (로봇 범위 기준)
        로봇의 가동 범위를 기어비율로 나눠서 텔레옵 범위 결정
        """
        robot_id = self.teleop_to_robot_mapping.get(teleop_id)
        if not robot_id:
            return 0, 4095

        gear_ratio = self.teleop_gear_ratio.get(teleop_id, 1)
        if gear_ratio == -1:  # 그리퍼는 전체 범위 허용
            return 0, 4095

        # 로봇의 범위와 기본자세
        robot_info = self.real_active_joints.get(robot_id, {})
        robot_min = robot_info.get('min', 0)
        robot_max = robot_info.get('max', 4095)
        robot_baseline = self.get_robot_baseline(robot_id)

        # 로봇이 기본자세에서 움직일 수 있는 범위
        robot_range_to_min = robot_baseline - robot_min  # min 방향으로
        robot_range_to_max = robot_max - robot_baseline  # max 방향으로

        # 텔레옵 기본자세
        teleop_baseline = self.get_teleop_baseline(teleop_id)

        # 텔레옵의 유효 범위 (기어비율 적용)
        teleop_range_to_min = robot_range_to_min / gear_ratio
        teleop_range_to_max = robot_range_to_max / gear_ratio

        teleop_effective_min = max(0, int(teleop_baseline - teleop_range_to_min))
        teleop_effective_max = min(4095, int(teleop_baseline + teleop_range_to_max))

        return teleop_effective_min, teleop_effective_max

    def normalize_delta(self, delta):
        """
        델타값을 최단 경로로 정규화 (-2048 ~ +2047 범위)
        예: 0에서 4090으로 가면 delta=3990이 아니라 delta=-6
        """
        delta = delta % 4096
        if delta > 2048:
            delta -= 4096
        elif delta < -2048:
            delta += 4096
        return delta

    def convert_teleop_to_robot(self, teleop_id, teleop_value):
        """
        텔레옵 위치를 로봇 위치로 변환
        """
        robot_id = self.teleop_to_robot_mapping.get(teleop_id)
        if not robot_id:
            return None

        gear_ratio = self.teleop_gear_ratio.get(teleop_id, 1)

        # 그리퍼 특별 처리 (58, 68번은 그리퍼)
        if teleop_id in [58, 68]:
            return self.convert_gripper_teleop_to_robot(teleop_id, teleop_value)

        # 유효 범위 체크: 0~4095 벗어나면 무시 (멀티턴 오류 방지)
        if teleop_value < 0 or teleop_value > 4095:
            return None

        # 텔레옵 기본자세 기준 변화량 (최단 경로 사용)
        teleop_baseline = self.get_teleop_baseline(teleop_id)
        teleop_delta = self.normalize_delta(teleop_value - teleop_baseline)

        # 기어비율 적용 (텔레옵 1 움직이면 로봇 gear_ratio 만큼 움직임)
        robot_delta = teleop_delta * gear_ratio

        # 로봇 목표 위치
        robot_baseline = self.get_robot_baseline(robot_id)
        robot_target = int(robot_baseline + robot_delta)

        # 오른팔 VR 매핑 오프셋 (vr_baseline - robot_regular)
        _vr_offset = {11: -1082, 12: -1214, 13: -1016, 14: 428, 15: 166, 16: 131}
        if robot_id in _vr_offset:
            robot_target += _vr_offset[robot_id]

        # 로봇 범위 제한
        robot_info = self.real_active_joints.get(robot_id, {})
        robot_min = robot_info.get('min', 0)
        robot_max = robot_info.get('max', 4095)
        robot_target = max(robot_min, min(robot_max, robot_target))

        return robot_target

    def convert_gripper_teleop_to_robot(self, teleop_id, teleop_value):
        """
        그리퍼 변환: 텔레옵 750 변화 = 로봇 4095 변화
        """
        robot_id = self.teleop_to_robot_mapping.get(teleop_id)
        if not robot_id:
            return None

        config = self.gripper_config.get(teleop_id)
        if not config:
            return None

        teleop_range = config["teleop_range"]   # 750
        robot_range = config["robot_range"]     # 4095
        direction = config["direction"]         # 1 또는 -1

        # 그리퍼 유효 범위 클램핑 (범위 초과 시 최대값으로 고정)
        # 왼손(68): 0~750 범위, 750 이상은 750으로
        # 오른손(58): 3345~4095 범위, 3345 이하는 3345로
        if teleop_id == 68:  # 왼손
            teleop_value = max(0, min(750, teleop_value))
        elif teleop_id == 58:  # 오른손
            teleop_value = max(4095 - 750, min(4095, teleop_value))

        # 텔레옵 기본자세 기준 변화량 (그리퍼는 정규화 없이 직접 계산)
        teleop_baseline = self.get_teleop_baseline(teleop_id)
        teleop_delta = teleop_value - teleop_baseline

        # 비율 계산: 텔레옵 750 -> 로봇 4095
        ratio = robot_range / teleop_range
        robot_delta = teleop_delta * ratio * direction

        # 로봇 목표 위치
        robot_baseline = self.get_robot_baseline(robot_id)
        robot_target = int(robot_baseline + robot_delta)

        # 로봇 범위 제한
        robot_target = max(0, min(4095, robot_target))

        return robot_target

    def _select_arm(self, arm):
        """팔 선택 토글 (All, Right, Left)"""
        self.teleop_arm_select.set(arm)

        # 버튼 색상 업데이트
        self.arm_btn_all.config(bg="gray")
        self.arm_btn_right.config(bg="gray")
        self.arm_btn_left.config(bg="gray")

        if arm == "All":
            self.arm_btn_all.config(bg="#4CAF50")  # 녹색
        elif arm == "Right":
            self.arm_btn_right.config(bg="#2196F3")  # 파란색
        elif arm == "Left":
            self.arm_btn_left.config(bg="#FF9800")  # 주황색

        self.log_real(f"[팔 선택] {arm}")

    def toggle_teleop_to_robot_sync(self):
        """텔레옵->로봇 연동 ON/OFF 토글"""
        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        if not self.real_connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다.")
            return

        # 카운트다운 중이면 취소
        if hasattr(self, 'countdown_active') and self.countdown_active:
            self.countdown_active = False
            self.teleop_sync_btn.config(text="연동 OFF", bg="gray")
            self.log_real("[연동] 카운트다운 취소됨")
            return

        # 이미 연동 중이면 OFF
        if self.teleop_to_robot_active:
            self.stop_teleop_sync()
            return

        # 연동 시작 전 30초 카운트다운
        self.countdown_active = True
        self.countdown_remaining = 30
        self.teleop_sync_btn.config(text=f"대기 {self.countdown_remaining}초", bg="orange")
        self.teleop_remaining_label.config(text=f"(시작까지 {self.countdown_remaining}초)", foreground="orange")
        self.log_real("[연동] 30초 후 연동 시작...")
        self.root.after(1000, self.countdown_tick)

    def countdown_tick(self):
        """카운트다운 타이머"""
        if not hasattr(self, 'countdown_active') or not self.countdown_active:
            self.teleop_remaining_label.config(text="")
            return

        self.countdown_remaining -= 1

        if self.countdown_remaining <= 0:
            # 카운트다운 완료 - 연동 시작
            self.countdown_active = False
            self.start_teleop_sync_actual()
        else:
            # 남은 시간 표시
            self.teleop_sync_btn.config(text=f"대기 {self.countdown_remaining}초", bg="orange")
            self.teleop_remaining_label.config(text=f"(시작까지 {self.countdown_remaining}초)", foreground="orange")
            self.root.after(1000, self.countdown_tick)

    def start_teleop_sync_actual(self):
        """실제 연동 시작 (카운트다운 후 호출)"""
        self.teleop_to_robot_active = True

        # 연동 시작 전에 텔레옵 토크 OFF (손으로 움직일 수 있게)
        self.log_real("[연동] 텔레옵 토크 OFF 설정 중...")
        for teleop_id in self.teleop_active_joints.keys():
            try:
                packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x03, 0x28, 0x00])
                checksum = (~(teleop_id + 0x04 + 0x03 + 0x28 + 0x00)) & 0xFF
                packet.append(checksum)
                self.teleop_ser.write(packet)
                self.teleop_ser.flush()
                time.sleep(0.002)
            except:
                pass
        # 토크 OFF 응답 버퍼 비우기
        time.sleep(0.05)
        self.teleop_ser.reset_input_buffer()
        self.teleop_torque_enabled.set(False)
        self.teleop_torque_btn.config(text="토크 OFF")
        self.log_real("[연동] 텔레옵 토크 OFF 완료")

        # 기본자세 다시 로드
        self.load_teleop_baseline()
        self.load_robot_baseline()

        self.teleop_sync_btn.config(text="연동 ON", bg="green")
        self.log_real("[연동] 텔레옵->로봇 연동 시작!")

        # 자동 종료 타이머 시작
        self.teleop_start_time = time.time()
        self.start_teleop_auto_stop_timer()

        # 연동 스레드 시작
        self.teleop_sync_thread = threading.Thread(target=self.teleop_to_robot_sync_loop, daemon=True)
        self.teleop_sync_thread.start()

    def teleop_to_robot_sync_loop(self):
        """텔레옵->로봇 실시간 동기화 루프"""
        # Hz 값 읽기
        try:
            sync_hz = int(self.sync_hz_var.get())
            sync_hz = max(1, min(1000, sync_hz))  # 1~1000Hz 제한
        except:
            sync_hz = 15

        # 읽기 자체가 32ms (16서보 × 2ms) 소요되므로 추가 sleep 최소화
        read_time = 0.032  # 예상 읽기 시간
        target_period = 1.0 / sync_hz
        sleep_time = max(0.001, target_period - read_time)  # 최소 1ms
        self.log_real(f"[연동] 동기화 루프 시작... ({sync_hz}Hz, sleep={sleep_time*1000:.1f}ms)")

        # 저장된 baseline 사용 (파일에서 로드된 값 유지)
        self.log_real(f"[연동] 저장된 baseline 사용: {self.teleop_baseline}")

        slider_update_counter = 0

        # 이전 위치 캐시 (읽기 실패 시 사용)
        last_teleop_positions = {}
        fail_count = 0

        loop_count = 0
        all_servo_ids = list(self.teleop_active_joints.keys())
        # 오른팔: 51-58, 왼팔: 61-68
        right_arm_teleop_ids = [51, 52, 53, 54, 55, 56, 57, 58]
        left_arm_teleop_ids = [61, 62, 63, 64, 65, 66, 67, 68]

        while self.teleop_to_robot_active and self.teleop_connected and self.real_connected:
            try:
                # 팔 선택에 따라 servo_ids 필터링
                arm_select = self.teleop_arm_select.get()
                if arm_select == "Right":
                    servo_ids = [sid for sid in all_servo_ids if sid in right_arm_teleop_ids]
                elif arm_select == "Left":
                    servo_ids = [sid for sid in all_servo_ids if sid in left_arm_teleop_ids]
                else:  # All
                    servo_ids = all_servo_ids

                # 1단계: 개별 읽기로 모든 텔레옵 위치 읽기 (Sync Read 미지원 서보)
                teleop_positions = {}
                for teleop_id in servo_ids:
                    teleop_pos = self.read_single_teleop_position(teleop_id)
                    if teleop_pos is not None:
                        teleop_positions[teleop_id] = teleop_pos
                        last_teleop_positions[teleop_id] = teleop_pos
                    elif teleop_id in last_teleop_positions:
                        teleop_positions[teleop_id] = last_teleop_positions[teleop_id]

                # 디버그: 처음 몇 번만 로그 출력
                loop_count += 1
                if loop_count <= 3:
                    self.log_real(f"[연동] 루프 {loop_count}: 읽은 텔레옵 수={len(teleop_positions)}개")

                # 2단계: 로봇 위치로 변환 (기본자세 잠금 시 기본자세 사용)
                # 로봇 ID 필터링 (팔 선택에 따라)
                right_arm_robot_ids = [11, 12, 13, 14, 15, 16, 17, 41]
                left_arm_robot_ids = [21, 22, 23, 24, 25, 26, 27, 31]

                robot_targets = {}
                if self.teleop_baseline_locked:
                    # 기본자세 잠금 상태: 모든 팔 기본자세 유지
                    for robot_id in self.robot_baseline.keys():
                        if robot_id not in self.real_active_joints:
                            continue
                        robot_targets[robot_id] = self.robot_baseline[robot_id]
                else:
                    # 정상 연동: 텔레옵 -> 로봇 변환
                    for teleop_id, teleop_pos in teleop_positions.items():
                        robot_target = self.convert_teleop_to_robot(teleop_id, teleop_pos)
                        if robot_target is not None:
                            robot_id = self.teleop_to_robot_mapping[teleop_id]
                            robot_targets[robot_id] = robot_target

                    # 선택하지 않은 팔은 기본자세로 고정
                    if arm_select == "Right":
                        # 왼팔은 기본자세로 고정
                        for robot_id in left_arm_robot_ids:
                            if robot_id in self.robot_baseline and robot_id in self.real_active_joints:
                                robot_targets[robot_id] = self.robot_baseline[robot_id]
                    elif arm_select == "Left":
                        # 오른팔은 기본자세로 고정
                        for robot_id in right_arm_robot_ids:
                            if robot_id in self.robot_baseline and robot_id in self.real_active_joints:
                                robot_targets[robot_id] = self.robot_baseline[robot_id]

                if loop_count <= 3:
                    locked_str = " (기본자세 잠금)" if self.teleop_baseline_locked else ""
                    self.log_real(f"[연동] 루프 {loop_count}: 로봇 타겟 수={len(robot_targets)}{locked_str}")

                # 3단계: Sync Write로 한 번에 전송
                if robot_targets:
                    self.sync_write_positions(robot_targets)

                # 3.5단계: 녹화 중이면 로봇 실제 위치 읽어서 캐시 (Sync Read)
                if self.is_recording:
                    positions = self.sync_read_positions(self.learning_joint_ids)
                    if positions:
                        self.cached_robot_positions = positions

                # 4단계: 슬라이더 업데이트 (10번에 1번만, GUI 스레드에서 실행)
                slider_update_counter += 1
                if slider_update_counter >= 10:
                    slider_update_counter = 0
                    # GUI 업데이트는 메인 스레드에서 (스레드 안전)
                    teleop_copy = dict(teleop_positions)
                    robot_copy = dict(robot_targets)
                    self.root.after(0, lambda t=teleop_copy, r=robot_copy: self._update_sync_sliders(t, r))

                time.sleep(sleep_time)

            except Exception as e:
                self.log_real(f"[연동] 오류: {e}")
                time.sleep(0.05)

        self.log_real("[연동] 동기화 루프 종료")

    def _update_sync_sliders(self, teleop_positions, robot_targets):
        """GUI 슬라이더 업데이트 (메인 스레드에서 호출)"""
        try:
            # 기본자세 잠금 시 슬라이더 업데이트 건너뜀 (콜백 트리거 방지)
            if self.teleop_baseline_locked:
                return

            for teleop_id, teleop_pos in teleop_positions.items():
                if teleop_id in self.teleop_sliders:
                    # 슬라이더 표시용 위치 (반전 적용)
                    display_pos = self.teleop_display_position(teleop_id, teleop_pos)
                    self.teleop_sliders[teleop_id].set(display_pos)
                robot_id = self.teleop_to_robot_mapping.get(teleop_id)
                if robot_id and robot_id in robot_targets and robot_id in self.real_sliders:
                    self.real_sliders[robot_id].set(robot_targets[robot_id])
        except:
            pass

    def stop_teleop_sync(self):
        """텔레옵 연동 중지"""
        self.teleop_to_robot_active = False
        self.teleop_sync_btn.config(text="연동 OFF", bg="gray")
        self.log_real("[연동] 텔레옵->로봇 연동 중지")

        # 텔레옵 Play 모드도 리셋
        if hasattr(self, '_teleop_play_active') and self._teleop_play_active:
            self._teleop_play_active = False
            self.teleop_play_btn.configure(text="▶ Teleop Play", bg='#ff8c00')

        # 자동 종료 타이머 취소
        if self.teleop_timer_id:
            self.root.after_cancel(self.teleop_timer_id)
            self.teleop_timer_id = None
        self.teleop_start_time = None
        self.teleop_remaining_label.config(text="")

    def start_teleop_auto_stop_timer(self):
        """텔레옵 자동 종료 타이머 시작"""
        # 이전 타이머 취소
        if self.teleop_timer_id:
            self.root.after_cancel(self.teleop_timer_id)
            self.teleop_timer_id = None

        # 자동 종료 시간 확인
        auto_stop = self.teleop_auto_stop_minutes.get()
        if auto_stop == "무한":
            self.teleop_remaining_label.config(text="(무한)")
            return

        try:
            minutes = int(auto_stop)
            if minutes <= 0:
                self.teleop_remaining_label.config(text="(무한)")
                return
        except:
            minutes = 20  # 기본값

        # 타이머 시작
        self.teleop_timer_id = self.root.after(1000, self.update_teleop_remaining_time)
        self.log_real(f"[연동] 자동 종료 타이머 시작: {minutes}분")

    def update_teleop_remaining_time(self):
        """텔레옵 남은 시간 업데이트 및 자동 종료"""
        if not self.teleop_to_robot_active or self.teleop_start_time is None:
            self.teleop_remaining_label.config(text="")
            return

        # 자동 종료 시간 확인
        auto_stop = self.teleop_auto_stop_minutes.get()
        if auto_stop == "무한":
            self.teleop_remaining_label.config(text="(무한)", foreground="green")
            self.teleop_timer_id = self.root.after(1000, self.update_teleop_remaining_time)
            return

        try:
            minutes = int(auto_stop)
        except:
            minutes = 20

        # 경과 시간 계산
        elapsed = time.time() - self.teleop_start_time
        remaining_seconds = (minutes * 60) - elapsed

        if remaining_seconds <= 0:
            # 시간 종료 - 연동 중지
            self.log_real(f"[연동] 자동 종료 ({minutes}분 경과)")
            self.stop_teleop_sync()
            return

        # 남은 시간 표시 (분:초)
        remaining_min = int(remaining_seconds // 60)
        remaining_sec = int(remaining_seconds % 60)

        # 1분 미만이면 빨간색, 아니면 녹색
        if remaining_seconds < 60:
            color = "red"
        else:
            color = "green"
        self.teleop_remaining_label.config(text=f"(남은시간 {remaining_min}:{remaining_sec:02d})", foreground=color)

        # 1초마다 업데이트
        self.teleop_timer_id = self.root.after(1000, self.update_teleop_remaining_time)

    def read_single_teleop_position(self, teleop_id, debug=False):
        """단일 텔레옵 서보 위치 읽기 (최적화, 스레드 안전)"""
        if not self.teleop_ser:
            return None

        # 락 획득 시도 (최대 100ms 대기)
        if not self.teleop_ser_lock.acquire(timeout=0.1):
            if debug:
                self.log_real(f"[DEBUG] ID {teleop_id}: 락 획득 실패")
            return None

        try:
            # Read Position 패킷 (주소 0x38, 길이 2)
            packet = bytearray([0xFF, 0xFF, teleop_id, 0x04, 0x02, 0x38, 0x02])
            checksum = (~(teleop_id + 0x04 + 0x02 + 0x38 + 0x02)) & 0xFF
            packet.append(checksum)

            self.teleop_ser.reset_input_buffer()
            self.teleop_ser.write(packet)
            self.teleop_ser.flush()
            time.sleep(0.001)  # 1ms 대기 (속도 향상)

            response = self.teleop_ser.read(12)  # 응답 패킷 크기
            if debug:
                self.log_real(f"[DEBUG] ID {teleop_id}: 응답 길이={len(response)}, 데이터={response.hex() if response else 'None'}")

            # 응답에서 FF FF ID 패턴 찾기
            for i in range(len(response) - 7):
                if response[i] == 0xFF and response[i+1] == 0xFF and response[i+2] == teleop_id:
                    pos_low = response[i+5]
                    pos_high = response[i+6]
                    position = pos_low + (pos_high << 8)
                    return position

        except Exception as e:
            if debug:
                self.log_real(f"[DEBUG] ID {teleop_id}: 예외={e}")

        finally:
            self.teleop_ser_lock.release()

        return None

    def teleop_display_position(self, teleop_id, raw_position):
        """텔레옵 슬라이더 표시용 위치 변환 (ROS Sim 방식 반전 적용)"""
        if teleop_id in self.teleop_display_inverted:
            return 4095 - raw_position
        return raw_position

    def teleop_raw_position(self, teleop_id, display_position):
        """텔레옵 표시 위치를 실제 위치로 역변환"""
        if teleop_id in self.teleop_display_inverted:
            return 4095 - display_position
        return display_position

    def sync_read_teleop_positions(self, servo_ids):
        """Sync Read로 여러 텔레옵 서보 위치 한 번에 읽기 (스레드 안전)"""
        if not self.teleop_ser:
            return {}

        # 락 획득 시도
        if not self.teleop_ser_lock.acquire(timeout=0.1):
            return {}

        positions = {}

        try:
            # Sync Read 패킷: FF FF FE LEN 0x82 START_ADDR DATA_LEN ID1 ID2 ... CHECKSUM
            # START_ADDR = 0x38 (현재 위치), DATA_LEN = 2
            start_addr = 0x38
            data_len = 2
            num_servos = len(servo_ids)

            # LEN = 4 + 서보 수
            packet_len = 4 + num_servos

            packet = bytearray([0xFF, 0xFF, 0xFE, packet_len, 0x82, start_addr, data_len])
            packet.extend(servo_ids)

            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            self.teleop_ser.reset_input_buffer()
            self.teleop_ser.write(packet)
            self.teleop_ser.flush()
            time.sleep(0.003)  # 3ms 대기 (16개 서보 응답 시간)

            # 각 서보 응답: FF FF ID LEN ERR DATA_L DATA_H CHECKSUM (8바이트)
            expected_len = num_servos * 8
            response = self.teleop_ser.read(expected_len + 16)  # 여유분 추가

            # 응답 파싱 (FF FF 패턴 찾기)
            idx = 0
            while idx + 7 <= len(response):
                if response[idx] == 0xFF and response[idx+1] == 0xFF:
                    servo_id = response[idx+2]
                    pkt_len = response[idx+3]
                    if pkt_len >= 4 and idx + 5 + pkt_len - 2 <= len(response):
                        err = response[idx+4]
                        if err == 0:  # 에러 없음
                            pos_low = response[idx+5]
                            pos_high = response[idx+6]
                            position = pos_low + (pos_high << 8)
                            if 0 <= position <= 4095:  # 유효 범위 체크
                                positions[servo_id] = position
                        idx += 4 + pkt_len  # 다음 패킷으로
                    else:
                        idx += 1
                else:
                    idx += 1

        except Exception as e:
            pass
        finally:
            self.teleop_ser_lock.release()

        return positions

    def sync_write_positions(self, servo_positions):
        """Sync Write - 한 번에 여러 서보에 위치 명령 전송"""
        if not self.real_connected or not self.real_ser:
            return False

        if not servo_positions:
            return False

        try:
            # Sync Write 패킷 구조:
            # FF FF FE LEN 0x83 START_ADDR DATA_LEN [ID1 DATA1_L DATA1_H] [ID2 DATA2_L DATA2_H] ... CHECKSUM
            # START_ADDR = 0x2A (목표 위치), DATA_LEN = 2 (2바이트)

            start_addr = 0x2A
            data_len = 2
            num_servos = len(servo_positions)

            # LEN = 4 + (DATA_LEN + 1) * 서보 수 = 4 + 3 * N
            packet_len = 4 + (data_len + 1) * num_servos

            packet = bytearray([0xFF, 0xFF, 0xFE, packet_len, 0x83, start_addr, data_len])

            for servo_id, position in servo_positions.items():
                pos = max(0, min(4095, int(position)))
                pos_low = pos & 0xFF
                pos_high = (pos >> 8) & 0xFF
                packet.extend([servo_id, pos_low, pos_high])

            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            self.real_ser.write(packet)
            self.real_ser.flush()
            return True

        except Exception as e:
            return False

    def create_status_panel(self, parent):
        """상태 패널"""
        status_frame = ttk.LabelFrame(parent, text="📡 상태", padding=5)
        status_frame.pack(fill=tk.X, pady=(10, 0))

        self.real_status_text = tk.Text(status_frame, height=6, font=("Courier", 10))
        self.real_status_text.pack(fill=tk.X)

        self.log_real("RRR Controller v1 시작됨")

    # ===================================================================
    # 서보 위치 Read-back (실제 모터 → GUI 슬라이더 반영)
    # ===================================================================

    def toggle_readback(self):
        """위치 읽기 토글"""
        if self.readback_running:
            self.stop_readback()
            self.readback_btn.config(text="위치읽기")
        else:
            if not self.real_connected:
                self.log_real("[Read-back] Real이 연결되지 않았습니다.")
                return
            self.start_readback()
            self.readback_btn.config(text="읽기중지")

    def start_readback(self):
        """서보 위치 읽기 루프 시작 (백그라운드 스레드)"""
        if self.readback_running:
            return
        self.readback_running = True
        self.log_real("[Read-back] 서보 위치 읽기 시작")
        threading.Thread(target=self._readback_worker, daemon=True).start()

    def stop_readback(self):
        """서보 위치 읽기 루프 중지"""
        self.readback_running = False
        self.log_real("[Read-back] 서보 위치 읽기 중지")

    def _readback_worker(self):
        """백그라운드 스레드: 서보 위치 읽기 (시리얼 I/O)"""
        while self.readback_running and self.real_connected:
            try:
                servo_ids = list(self.real_active_joints.keys())
                if not servo_ids:
                    time.sleep(0.1)
                    continue

                positions = {}
                for sid in servo_ids:
                    if not self.readback_running:
                        return
                    pos = self.read_real_servo_position(sid)
                    if pos is not None and 0 <= pos <= 4095:
                        positions[sid] = pos

                if positions:
                    # GUI 업데이트는 메인 스레드에서
                    self.root.after(0, lambda p=dict(positions): self._readback_update_gui(p))

                    # 네트워크 전송 (백그라운드에서 바로)
                    if self.network_connected:
                        try:
                            net_positions = {str(sid): p for sid, p in positions.items()}
                            msg = json.dumps({'type': 'all_sliders', 'positions': net_positions}) + '\n'
                            self.network_client_socket.sendall(msg.encode('utf-8'))
                        except Exception:
                            self.network_connected = False

            except Exception:
                pass

            time.sleep(0.05)  # 50ms 간격

        self.readback_running = False

    def _readback_update_gui(self, positions):
        """메인 스레드: 슬라이더/라벨 업데이트"""
        self.loading_baseline = True
        for servo_id, pos in positions.items():
            self.real_active_joints[servo_id]['current'] = pos
            if servo_id in self.real_sliders:
                self.real_sliders[servo_id].set(pos)
            if servo_id in self.real_value_labels:
                self.real_value_labels[servo_id].config(text=f"{pos}")
        self.loading_baseline = False

    # ===================================================================
    # 네트워크 동기화 (v7↔v8 GUI via Tailscale)
    # ===================================================================

    def toggle_network_connection(self):
        """네트워크 연결/해제 토글"""
        if self.network_connected:
            self.stop_network()
        else:
            self.connect_to_sim_server()

    def connect_to_sim_server(self):
        """Windows(Sim) 서버에 연결"""
        if self.network_connected:
            self.log_real("[NET] 이미 연결되어 있습니다.")
            return

        try:
            self.network_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.network_client_socket.settimeout(5.0)
            self.log_real(f"[NET] {self.network_server_ip}:{self.network_port} 연결 중...")
            self.network_client_socket.connect((self.network_server_ip, self.network_port))
            self.network_client_socket.settimeout(1.0)
            self.network_connected = True
            self.network_running = True

            self.log_real(f"[NET] 서버 연결 성공!")
            self.network_connect_btn.config(text="NET 해제")
            self.network_status_label.config(text="[연결됨]", foreground="green")

            # 수신 스레드 시작
            self.network_recv_thread = threading.Thread(target=self._network_receive_loop, daemon=True)
            self.network_recv_thread.start()

            # Real->Sim 모드면 전송 루프 시작
            if self.real_link_to_sim:
                self._start_network_sync_loop()

        except Exception as e:
            self.log_real(f"[NET ERR] 연결 실패: {e}")
            self.network_client_socket = None

    def _network_receive_loop(self):
        """데이터 수신 루프"""
        buffer = b''
        while self.network_running and self.network_connected:
            try:
                data = self.network_client_socket.recv(4096)
                if not data:
                    break

                buffer += data

                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        msg = json.loads(line.decode('utf-8'))
                        self.root.after(0, lambda m=msg: self._handle_network_message(m))
                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue
            except Exception as e:
                if self.network_running:
                    self.root.after(0, lambda: self.log_real(f"[NET ERR] 수신: {e}"))
                break

        self.network_connected = False
        self.root.after(0, lambda: self._on_network_disconnected())

    def _on_network_disconnected(self):
        """연결 끊김 처리"""
        self.log_real("[NET] 연결 종료됨")
        self.network_connect_btn.config(text="NET 연결")
        self.network_status_label.config(text="[X]", foreground="red")

    def _handle_network_message(self, msg):
        """수신된 메시지 처리"""
        try:
            msg_type = msg.get('type', '')

            if msg_type == 'all_sliders':
                positions = msg.get('positions', {})
                self._network_updating = True

                for servo_id_str, pos in positions.items():
                    servo_id = int(servo_id_str)

                    # Sim->Real 모드: Windows에서 받은 값으로 슬라이더 업데이트
                    if self.sim_link_to_real:
                        if servo_id in self.real_sliders:
                            self.real_sliders[servo_id].set(pos)
                            self.real_active_joints[servo_id]['current'] = pos
                            if self.real_connected:
                                self.send_real_servo_command(servo_id, pos)

                self._network_updating = False

            elif msg_type == 'command':
                cmd = msg.get('cmd', '')
                if cmd == 'default_pose':
                    self.root.after(0, self.move_to_default_pose)

        except Exception as e:
            self.log_real(f"[NET ERR] 메시지 처리: {e}")

    def send_network_sliders(self):
        """슬라이더 값 전송"""
        if not self.network_connected or self._network_updating:
            return

        now = time.time()
        if now - self.network_last_send_time < self.network_send_interval:
            return
        self.network_last_send_time = now

        try:
            positions = {}
            if self.real_link_to_sim:
                for servo_id, info in self.real_active_joints.items():
                    positions[str(servo_id)] = info.get('current', 0)

            if positions:
                msg = {'type': 'all_sliders', 'positions': positions}
                data = json.dumps(msg) + '\n'
                self.network_client_socket.sendall(data.encode('utf-8'))

        except Exception as e:
            self.log_real(f"[NET ERR] 전송: {e}")
            self.network_connected = False

    def _start_network_sync_loop(self):
        """Real->Sim 모드: 주기적으로 슬라이더 값 전송"""
        if not self.real_link_to_sim:
            return
        if self.network_connected:
            self.send_network_sliders()
        if self.real_link_to_sim and self.network_running:
            self.root.after(100, self._start_network_sync_loop)

    def stop_network(self):
        """네트워크 연결 종료"""
        self.network_running = False
        self.network_connected = False

        if self.network_client_socket:
            try:
                self.network_client_socket.close()
            except:
                pass
            self.network_client_socket = None

        self.log_real("[NET] 네트워크 종료됨")
        self.network_connect_btn.config(text="NET 연결")
        self.network_status_label.config(text="[X]", foreground="red")

    # ===================================================================
    # 연동 모드 제어
    # ===================================================================

    def change_sync_mode(self):
        """연동 모드 변경"""
        mode = self.sync_mode.get()
        self.log_real(f"[DEBUG] change_sync_mode 호출됨, mode={mode}")

        if mode == "독립":
            self.real_link_to_sim = False
            self.sim_link_to_real = False
            self.log_real("[모드] 독립 모드 - Real과 Sim 따로 동작")

        elif mode == "Real->Sim":
            self.real_link_to_sim = True
            self.sim_link_to_real = False
            self.log_real("[모드] Real->Sim - 실제 모터 위치를 Sim이 따라옴")

            # 네트워크 연결되어 있으면 전송 루프 시작
            if self.network_connected:
                self.log_real("[NET] Real->Sim 네트워크 전송 시작")
                self._start_network_sync_loop()

            # 로컬 Sim 연동
            if self.real_connected:
                self.update_real_positions_to_sim()

        elif mode == "Sim->Real":
            self.real_link_to_sim = False
            self.sim_link_to_real = True
            self.log_real("[모드] Sim->Real - Sim이 Real 제어")
            if not self.real_connected:
                self.log_real("[경고] Real이 연결되지 않았습니다. Real을 먼저 연결하세요.")
            if not self.sim_connected:
                self.log_real("[경고] Sim이 연결되지 않았습니다. Sim을 먼저 시작하세요.")
            if self.real_connected and self.sim_connected:
                self.log_real("[AUTO] Sim->Real 모드 자동 시작")
                # 주기적으로 Sim 값을 읽어서 Real 모터 제어 시작
                self.update_sim_positions_to_real()

    # ===================================================================
    # Real 모드 로직
    # ===================================================================

    def toggle_real_connection(self):
        """Real 연결 토글"""
        if not self.real_connected:
            self.connect_real()
        else:
            self.disconnect_real()

    def find_servo_ports(self):
        """서보가 연결된 USB 포트 자동 감지"""
        import glob

        found_ports = []
        available_ports = glob.glob('/dev/ttyUSB*')

        self.log_real(f"[스캔] USB 포트 검색 중... {available_ports}")

        for port in available_ports:
            try:
                test_ser = serial.Serial(port, 1000000, timeout=0.02)
                time.sleep(0.05)

                # 서보 PING 테스트 (로봇 ID)
                test_ids = [11, 12, 21, 22, 31, 41]
                servo_found = False

                for servo_id in test_ids:
                    packet = bytearray([0xFF, 0xFF, servo_id, 0x02, 0x01])
                    checksum = (~(servo_id + 0x02 + 0x01)) & 0xFF
                    packet.append(checksum)

                    test_ser.write(packet)
                    test_ser.flush()
                    time.sleep(0.01)

                    response = test_ser.read(20)
                    if len(response) >= 6:
                        servo_found = True
                        self.log_real(f"[스캔] {port}: 서보 ID {servo_id} 발견!")
                        break

                test_ser.close()

                if servo_found:
                    found_ports.append(port)

            except Exception as e:
                self.log_real(f"[스캔] {port} 오류: {e}")

        return found_ports

    def connect_real(self):
        """Real 연결 - 자동 포트 감지"""
        try:
            # 자동으로 서보 포트 찾기
            found_ports = self.find_servo_ports()

            if not found_ports:
                self.log_real("[경고] 서보 포트 자동 감지 실패, /dev/ttyUSB0 시도...")
                found_ports = ['/dev/ttyUSB0']

            main_port = found_ports[0]
            self.real_ser = serial.Serial(main_port, 1000000, timeout=0.1)
            self.real_connected = True

            self.real_connect_btn.config(text="연결해제")
            self.real_status_label.config(text="[O] 연결됨", foreground="green")

            self.log_real(f"Real 로봇 연결 성공 ({main_port})")

            # 연결 상태 모니터링 시작 (3초마다)
            self._heartbeat_running = True
            threading.Thread(target=self._heartbeat_monitor, daemon=True).start()

            # 자동으로 기본 자세로 이동 (auto_real_baseline 내에서 ACC 적용됨)
            threading.Thread(target=self.auto_real_baseline, daemon=True).start()

        except Exception as e:
            messagebox.showerror("Real 연결 오류", f"연결 실패:\n{e}")
            self.log_real(f"연결 실패: {e}")

    def auto_real_baseline(self):
        """Real 연결 시 자동 기본 자세"""
        try:
            time.sleep(0.5)

            # 저장된 ACC 값 자동 적용
            acc_value = self.real_acceleration_value.get()
            gripper_acc = 20
            self.log_real(f"ACC 적용 중: {acc_value} (그리퍼: {gripper_acc})")

            for servo_id in self.real_active_joints.keys():
                if self.real_connected:
                    if servo_id in [41, 31]:
                        self.set_real_servo_acceleration(servo_id, gripper_acc)
                    else:
                        self.set_real_servo_acceleration(servo_id, acc_value)
                    time.sleep(0.01)

            self.log_real(f"[OK] ACC 설정 완료")

            # 기본 자세로 이동
            for servo_id, joint_info in self.real_active_joints.items():
                if self.real_connected:
                    self._safe_send_servo(servo_id, joint_info['current'])
                    time.sleep(0.05)
            self.log_real("[OK] 기본 자세로 이동 완료")
        except Exception as e:
            self.log_real(f"[ERR] 기본 자세 이동 실패: {e}")

    def disconnect_real(self):
        """Real 연결 해제"""
        self._heartbeat_running = False
        if self.real_ser and self.real_ser.is_open:
            self.real_ser.close()

        self.real_connected = False
        self.real_connect_btn.config(text="로봇 연결")
        self.real_status_label.config(text="[X] 연결안됨", foreground="red")

        self.log_real("Real 연결 해제됨")

    def _heartbeat_monitor(self):
        """3초마다 서보 응답을 확인하여 연결 끊김 감지"""
        fail_count = 0
        while self._heartbeat_running and self.real_connected:
            time.sleep(3)
            if not self._heartbeat_running or not self.real_connected:
                break
            try:
                # 서보 11번 위치 읽기 시도 (항상 존재하는 서보)
                pos = self.read_real_servo_position(11)
                if pos is not None:
                    fail_count = 0
                else:
                    fail_count += 1
            except Exception:
                fail_count += 1

            if fail_count >= 2:
                # 2회 연속 실패 → 연결 끊김 판정
                self._heartbeat_running = False
                self.real_connected = False
                self.root.after(0, lambda: self.real_connect_btn.config(text="로봇 연결"))
                self.root.after(0, lambda: self.real_status_label.config(text="[X] 연결 끊김", foreground="red"))
                self.root.after(0, lambda: self.log_real("[경고] 로봇 연결 끊김 감지! (서보 응답 없음)"))
                try:
                    if self.real_ser and self.real_ser.is_open:
                        self.real_ser.close()
                except Exception:
                    pass
                break

    # 구식 함수 - 더 이상 사용하지 않음 (라디오 버튼으로 대체됨)
    # def toggle_real_link_to_sim(self):
    #     pass

    # ===================================================================
    # ===================================================================

    def _apply_acceleration_settings(self):
        """가속 설정 적용 (버튼 동작 전 자동 호출)"""
        if not self.real_connected:
            return
        try:
            acc_value = self.real_acceleration_value.get()
            gripper_acc = 20
            for servo_id in self.real_active_joints.keys():
                if servo_id in [41, 31]:
                    self.set_real_servo_acceleration(servo_id, gripper_acc)
                else:
                    self.set_real_servo_acceleration(servo_id, acc_value)
                time.sleep(0.01)
            self.log_real(f"[ACC] 가속 설정 적용: {acc_value} (그리퍼: {gripper_acc})")
        except Exception as e:
            self.log_real(f"[ACC] 가속 설정 오류: {e}")

    def _safe_send_servo(self, servo_id, target_pos):
        """서보 명령 전송 (그리퍼는 현재 위치 읽고 가까우면 스킵)"""
        GRIPPER_IDS = [41, 31, 32, 28]
        if servo_id in GRIPPER_IDS:
            cur = self.read_real_servo_position(servo_id)
            if cur is not None:
                diff = abs(cur - target_pos)
                if diff < 50:
                    return  # 이미 가까움 → 스킵
                # 래핑 방지: 현재 위치가 범위 밖이면 가까운 방향으로 먼저 이동
                if cur > 4090 and target_pos < 100:
                    # 4095 근처 → 0 근처로 갈 때, 중간값으로 먼저 이동
                    self.send_real_servo_command(servo_id, 2048)
                    time.sleep(0.3)
                elif cur < 5 and target_pos > 3990:
                    self.send_real_servo_command(servo_id, 2048)
                    time.sleep(0.3)
        self.send_real_servo_command(servo_id, target_pos)

    def move_real_to_baseline(self):
        """Real 기본 자세로 이동 (엘보우 안전 동작 포함)"""
        self._apply_acceleration_settings()
        self.active_baseline_type = 'robot'
        # 베이스라인 로드
        self.load_real_baseline()
        # IK solver 재초기화 (robot baseline 기준)
        try:
            self.init_ik_solver()
        except Exception:
            pass

        # 로봇 연결 안되어도 슬라이더와 Sim은 업데이트
        if not self.real_connected:
            self.log_real("[Sim] 기본 자세로 이동 (로봇 미연결)")
            # 슬라이더 업데이트
            for servo_id, joint_info in self.real_active_joints.items():
                if servo_id in self.real_sliders:
                    self.real_sliders[servo_id].set(joint_info['current'])
                    self.real_value_labels[servo_id].config(text=f"{joint_info['current']}")
                if servo_id in self.sim_active_joints:
                    self.sim_active_joints[servo_id]['current'] = joint_info['current']
            # Sim 업데이트
            self.schedule_sim_update()
            # IK 슬라이더도 FK로 업데이트 (오류 시 무시)
            try:
                if self.ik_solver:
                    self._update_ik_sliders_from_fk('right')
                    self._update_ik_sliders_from_fk('left')
            except Exception:
                pass
            self.log_real("[Sim] 기본 자세 적용 완료")
            return

        # 로봇 연결된 경우: 기존 하드웨어 동작
        # 1단계: 기본자세로 이동
        self.log_real("[1/3] 기본 자세로 이동 중...")
        for servo_id, joint_info in self.real_active_joints.items():
            self._safe_send_servo(servo_id, joint_info['current'])
            time.sleep(0.02)

        time.sleep(0.5)  # 잠시 대기

        # 2단계: 엘보우 이동 (L +300, R -300)
        self.log_real("[2/3] 엘보우 안전 위치로 이동...")
        left_elbow_id = 24
        right_elbow_id = 14

        left_baseline = self.real_active_joints[left_elbow_id]['current']
        right_baseline = self.real_active_joints[right_elbow_id]['current']

        left_target = min(4095, left_baseline + 300)
        right_target = max(0, right_baseline - 300)

        self.send_real_servo_command(left_elbow_id, left_target)
        self.send_real_servo_command(right_elbow_id, right_target)

        time.sleep(0.5)  # 잠시 대기

        # 3단계: 다시 기본자세로 복귀
        self.log_real("[3/3] 기본 자세로 복귀...")
        self.send_real_servo_command(left_elbow_id, left_baseline)
        self.send_real_servo_command(right_elbow_id, right_baseline)

        # 모든 슬라이더 업데이트 (기본자세 값으로)
        for servo_id, joint_info in self.real_active_joints.items():
            if servo_id in self.real_sliders:
                self.real_sliders[servo_id].set(joint_info['current'])
                self.real_value_labels[servo_id].config(text=f"{joint_info['current']}")
            if servo_id in self.sim_active_joints:
                self.sim_active_joints[servo_id]['current'] = joint_info['current']

        # Sim 업데이트
        self.schedule_sim_update()

        # IK 슬라이더도 FK로 업데이트 (오류 시 무시)
        try:
            if self.ik_solver:
                self._update_ik_sliders_from_fk('right')
                self._update_ik_sliders_from_fk('left')
        except Exception:
            pass

        self.log_real("기본 자세로 이동 완료 (엘보우 안전 동작 포함)")

    def move_real_to_vr_baseline(self):
        """VR 텔레옵 전용 기본자세로 이동 (팔을 더 내린 자세)"""
        self._apply_acceleration_settings()
        self.log_real("[VR] VR 기본자세로 이동 중...")
        self.active_baseline_type = 'vr'

        # 슬라이더 및 Sim 업데이트
        for servo_id, position in self.vr_baseline.items():
            if servo_id in self.real_active_joints:
                self.real_active_joints[servo_id]['current'] = position
                if servo_id in self.real_sliders:
                    self.real_sliders[servo_id].set(position)
                    self.real_value_labels[servo_id].config(text=f"{position}")
                if servo_id in self.sim_active_joints:
                    self.sim_active_joints[servo_id]['current'] = position

        # 로봇 연결된 경우 실제 서보도 이동
        if self.real_connected:
            for servo_id, position in self.vr_baseline.items():
                if servo_id in self.real_active_joints:
                    self._safe_send_servo(servo_id, position)
                    time.sleep(0.02)

        # Sim 업데이트
        self.schedule_sim_update()

        # IK solver를 VR baseline 기준으로 재초기화
        # VR baseline 그리퍼 위치가 IK의 (0,0,0) 기준점이 됨
        try:
            # IK solver 재초기화 (VR baseline 기준, VIZ_BASELINE_OVERRIDE 사용 안 함)
            self.init_ik_solver()
            self.ik_enabled.set(True)

            # IK 슬라이더 0으로 리셋 (VR baseline이 새 기준점)
            self.read_current_ik_position()
            self.log_real("[VR] IK solver가 VR 기본자세 기준으로 재초기화됨")
        except Exception as e:
            self.log_real(f"[VR] IK solver 재초기화 실패: {e}")

        self.log_real("[VR] VR 기본자세 이동 완료")

    def move_to_min_load_pose(self):
        """최소 부하 자세로 이동 (V3 기본자세 - 모터 부하 최소화)"""
        # V3 기본자세 값 (하드코딩)
        min_load_pose = {
            11: 2964, 12: 92, 13: 1923, 14: 4095, 15: 2995, 16: 2006, 17: 2980, 41: 0,
            21: 1194, 22: 3906, 23: 2006, 24: 0, 25: 2112, 26: 2112, 27: 1854, 31: 0,
            32: 0, 28: 0
        }

        # 슬라이더 업데이트 (로봇 연결 여부와 관계없이)
        for servo_id, position in min_load_pose.items():
            if servo_id in self.real_active_joints:
                self.real_active_joints[servo_id]['current'] = position
                if servo_id in self.real_sliders:
                    self.real_sliders[servo_id].set(position)
                    self.real_value_labels[servo_id].config(text=f"{position}")
                if servo_id in self.sim_active_joints:
                    self.sim_active_joints[servo_id]['current'] = position

        # 로봇 연결된 경우: 하드웨어로 명령 전송
        if self.real_connected:
            for servo_id, position in min_load_pose.items():
                if servo_id in self.real_active_joints:
                    self.send_real_servo_command(servo_id, position)
                    time.sleep(0.02)
            self.log_real("최소 부하 자세로 이동 완료 (V3 기본자세)")
        else:
            self.log_real("[Sim] 최소 부하 자세 적용 완료 (로봇 미연결)")

        # Sim 업데이트
        self.schedule_sim_update()

        # IK 슬라이더도 FK로 업데이트 (오류 시 무시)
        try:
            if self.ik_solver:
                self._update_ik_sliders_from_fk('right')
                self._update_ik_sliders_from_fk('left')
        except Exception:
            pass

    def load_real_baseline(self):
        """Real 베이스라인 로드"""
        try:
            # 베이스라인 로드 시작 - 슬라이더 콜백에서 update_sim 방지
            self.loading_baseline = True

            with open(self.robot_baseline_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            for servo_id in self.real_active_joints.keys():
                if str(servo_id) in data['servos']:
                    servo_data = data['servos'][str(servo_id)]
                    baseline_pos = servo_data['position']
                    # 그리퍼 서보 10~4035 클램핑
                    if servo_id in [41, 31, 32, 28]:
                        baseline_pos = max(10, min(4035, baseline_pos))
                    self.real_baseline_positions[servo_id] = baseline_pos
                    self.real_active_joints[servo_id]['current'] = baseline_pos

                    # Sim 베이스라인과 current 모두 동기화
                    if servo_id in self.sim_active_joints:
                        self.sim_baseline_positions[servo_id] = baseline_pos
                        self.sim_active_joints[servo_id]['current'] = baseline_pos

                    if servo_id in self.real_sliders:
                        self.real_sliders[servo_id].set(baseline_pos)
                        self.real_value_labels[servo_id].config(text=f"{baseline_pos}")

                    # Sim 반전 상태 로드 및 버튼 UI 업데이트
                    if 'sim_inverted' in servo_data:
                        is_inverted = servo_data['sim_inverted']
                        self.sim_inverted[servo_id] = is_inverted
                        if servo_id in self.invert_buttons:
                            if is_inverted:
                                self.invert_buttons[servo_id].config(text="[V] Sim반전")
                            else:
                                self.invert_buttons[servo_id].config(text="[ ] Sim반전")

            self.log_real("Real 베이스라인 로드 완료 (Sim 반전 설정 포함)")

            # 베이스라인 로드 완료 - 플래그 해제
            self.loading_baseline = False

            # Sim이 연결되어 있으면 한 번만 업데이트
            if self.sim_connected and self.real_link_to_sim:
                self.update_sim()

        except Exception as e:
            self.loading_baseline = False  # 에러 발생 시에도 플래그 해제
            self.log_real(f"Real 베이스라인 로드 실패: {e}")

    def save_current_pose(self):
        """현재 자세를 기본 자세로 저장"""
        try:
            # 현재 JSON 파일 읽기
            baseline_path = self.robot_baseline_file  # 절대 경로 사용
            backup_path = baseline_path.replace('.json', '_backup.json')

            # 기존 baseline 백업 생성
            import shutil
            if os.path.exists(baseline_path):
                shutil.copy2(baseline_path, backup_path)
                self.log_real(f"[INFO] 기존 기본 자세 백업됨: {backup_path}")

            with open(baseline_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            NO_GEARBOX_SERVOS = [15, 25]  # 360도 회전 가능 (기어비 없음)

            # 현재 슬라이더 값으로 업데이트
            for servo_id, joint_info in self.real_active_joints.items():
                current_pos = joint_info['current']

                # 서보 15, 25는 360도 범위, 나머지는 120도 범위
                if servo_id in NO_GEARBOX_SERVOS:
                    angle_deg = (current_pos / 4095) * 360
                else:
                    angle_deg = (current_pos / 4095) * 120

                data['servos'][str(servo_id)] = {
                    "position": current_pos,
                    "angle_degrees": round(angle_deg, 2),
                    "status": "OK",
                    "sim_inverted": self.sim_inverted.get(servo_id, False)
                }

            # 메타데이터 업데이트 (없으면 생성)
            if 'metadata' not in data:
                data['metadata'] = {}
            data['metadata']['timestamp'] = time.strftime("%Y-%m-%d %H:%M:%S")
            data['metadata']['description'] = "GUI에서 설정한 새로운 기본 자세"

            # JSON 파일에 저장
            with open(baseline_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            self.log_real("[OK] 현재 자세가 기본 자세로 저장되었습니다")
            messagebox.showinfo("자세 저장", "현재 자세가 기본 자세로 저장되었습니다!")

        except Exception as e:
            self.log_real(f"[ERR] 자세 저장 실패: {e}")
            messagebox.showerror("저장 오류", f"자세 저장 실패:\n{e}")

    def toggle_sim_invert(self, servo_id):
        """Sim 반전 토글 (Isaac Sim에만 적용)"""
        # 반전 상태 토글
        self.sim_inverted[servo_id] = not self.sim_inverted[servo_id]

        # 버튼 텍스트 업데이트
        if self.sim_inverted[servo_id]:
            self.invert_buttons[servo_id].config(text="[V] Sim반전")
            self.log_real(f"[Sim] 서보 {servo_id} 반전 활성화")
        else:
            self.invert_buttons[servo_id].config(text="[ ] Sim반전")
            self.log_real(f"[Sim] 서보 {servo_id} 반전 비활성화")

        # Sim이 연결되어 있으면 즉시 업데이트
        if self.sim_connected and self.real_link_to_sim:
            self.update_sim()

    def set_real_servo_acceleration(self, servo_id, acc_value):
        """Real 서보 모터의 ACC 설정"""
        if not self.real_connected or not self.real_ser:
            return False

        try:
            actual_acc = max(0, min(254, acc_value))
            packet = [0xFF, 0xFF, servo_id, 0x04, 0x03, 0x29, actual_acc]
            checksum = (~sum(packet[2:]) % 256) & 0xFF

            self.real_ser.write(bytes(packet + [checksum]))
            time.sleep(0.005)
            return True
        except Exception as e:
            self.log_real(f"서보 {servo_id} ACC 설정 실패: {e}")
            return False

    def send_real_servo_command(self, servo_id, position):
        """Real 서보 모터에 위치 명령 전송 (STS3215 프로토콜)"""
        if not self.real_connected or not self.real_ser:
            return False

        try:
            # 위치 값을 0-4095 범위로 제한
            actual_pos = max(0, min(4095, int(position)))

            # 위치를 low/high byte로 분리
            pos_low = actual_pos & 0xFF
            pos_high = (actual_pos >> 8) & 0xFF

            # STS3215 Write 명령 (원본 Windows 코드와 동일)
            # FF FF ID LEN CMD ADDR POS_LOW POS_HIGH CHECKSUM
            # LEN=0x05, CMD=0x03 (Write), ADDR=0x2A (목표 위치 레지스터)
            packet = [0xFF, 0xFF, servo_id, 0x05, 0x03, 0x2A, pos_low, pos_high]
            checksum = (~sum(packet[2:]) % 256) & 0xFF

            self.real_ser.write(bytes(packet + [checksum]))

            return True
        except Exception as e:
            self.log_real(f"서보 {servo_id} 위치 명령 실패: {e}")
            return False

    def read_real_servo_position(self, servo_id):
        """Real 서보 모터의 현재 위치 읽기 (STS3215 프로토콜)"""
        if not self.real_connected or not self.real_ser:
            return None

        try:
            # READ 명령으로 현재 위치 읽기
            # FF FF ID LEN CMD ADDR DATA_LEN CHECKSUM
            # LEN=0x04, CMD=0x02 (Read), ADDR=0x38 (현재 위치 레지스터), DATA_LEN=0x02 (2바이트)
            packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
            checksum = (~sum(packet[2:]) % 256) & 0xFF

            self.real_ser.reset_input_buffer()
            self.real_ser.write(bytes(packet + [checksum]))
            time.sleep(0.005)

            response = self.real_ser.read(100)

            if len(response) >= 8:
                pos_l = response[5]
                pos_h = response[6]
                position = pos_l + (pos_h << 8)
                return position
            return None
        except Exception as e:
            return None

    def sync_read_positions(self, servo_ids):
        """여러 서보의 위치를 한 번에 읽기 (Sync Read - 0x82)

        Args:
            servo_ids: 읽을 서보 ID 리스트

        Returns:
            dict: {servo_id: position} 또는 실패 시 빈 딕셔너리
        """
        if not self.real_connected or not self.real_ser:
            return {}

        try:
            # Sync Read 패킷: FF FF FE [Length] 82 [MemAddr] [DataLen] [ID1] [ID2] ... [Checksum]
            # MemAddr = 0x38 (현재 위치), DataLen = 0x02 (2바이트)
            broadcast_id = 0xFE
            instruction = 0x82  # SYNC_READ
            mem_addr = 0x38     # 현재 위치 레지스터
            data_len = 0x02     # 2바이트 (위치값)

            # Length = 서보 개수 + 4
            length = len(servo_ids) + 4

            # 패킷 구성
            packet = [0xFF, 0xFF, broadcast_id, length, instruction, mem_addr, data_len]
            packet.extend(servo_ids)

            # 체크섬 계산
            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            # 전송
            self.real_ser.reset_input_buffer()
            self.real_ser.write(bytes(packet))

            # 응답 대기 (각 서보당 8바이트 응답)
            expected_response_len = len(servo_ids) * 8
            time.sleep(0.005 + len(servo_ids) * 0.002)  # 기본 5ms + 서보당 2ms

            response = self.real_ser.read(expected_response_len + 50)  # 여유분 포함

            # 응답 파싱
            positions = {}
            idx = 0
            for servo_id in servo_ids:
                # 각 서보 응답: FF FF [ID] [Length] [Error] [Data_L] [Data_H] [Checksum]
                # 응답에서 해당 서보의 데이터 찾기
                while idx < len(response) - 7:
                    if response[idx] == 0xFF and response[idx+1] == 0xFF:
                        resp_id = response[idx+2]
                        if resp_id == servo_id:
                            pos_l = response[idx+5]
                            pos_h = response[idx+6]
                            position = pos_l + (pos_h << 8)
                            if 0 <= position <= 4095:
                                positions[servo_id] = position
                            idx += 8
                            break
                    idx += 1

            return positions

        except Exception as e:
            return {}

    def read_servo_temperature(self, servo_id):
        """STS3215 서보 온도 읽기 (레지스터 0x3F)"""
        if not self.real_connected or not self.real_ser:
            return None

        try:
            # READ 명령: FF FF ID LEN CMD ADDR DATA_LEN CHECKSUM
            # ADDR=0x3F (온도), DATA_LEN=0x01
            packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x3F, 0x01]
            checksum = (~sum(packet[2:]) % 256) & 0xFF

            self.real_ser.reset_input_buffer()
            self.real_ser.write(bytes(packet + [checksum]))
            time.sleep(0.005)

            response = self.real_ser.read(100)

            if len(response) >= 7:
                temp = response[5]  # 온도 (°C)
                return temp
            return None
        except Exception as e:
            return None

    def read_all_servo_temperatures(self):
        """모든 서보 온도 읽기"""
        temps = {}
        for servo_id in self.real_active_joints.keys():
            temp = self.read_servo_temperature(servo_id)
            if temp is not None:
                temps[servo_id] = temp
        return temps

    def show_servo_temperatures(self):
        """서보 온도 확인 및 표시"""
        if not self.real_connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다.")
            return

        self.log_real("[온도] 서보 온도 읽는 중...")

        temps = self.read_all_servo_temperatures()

        if not temps:
            self.log_real("[온도] 온도 읽기 실패")
            messagebox.showerror("오류", "서보 온도를 읽을 수 없습니다.")
            return

        # 온도 정보 표시
        self.log_real("=" * 40)
        self.log_real("서보 모터 온도 현황")
        self.log_real("=" * 40)

        right_arm = []
        left_arm = []
        max_temp = 0
        max_temp_id = 0

        for servo_id, temp in sorted(temps.items()):
            name = self.real_active_joints.get(servo_id, {}).get('name', f'ID{servo_id}')
            status = "정상" if temp < 50 else "주의" if temp < 60 else "위험!"

            if temp > max_temp:
                max_temp = temp
                max_temp_id = servo_id

            line = f"  {servo_id:2d} ({name}): {temp}°C [{status}]"
            self.log_real(line)

            if servo_id < 20 or servo_id == 41:
                right_arm.append((servo_id, temp))
            else:
                left_arm.append((servo_id, temp))

        self.log_real("-" * 40)
        max_name = self.real_active_joints.get(max_temp_id, {}).get('name', f'ID{max_temp_id}')
        self.log_real(f"최고 온도: {max_temp}°C (ID {max_temp_id}: {max_name})")
        self.log_real("=" * 40)

        # 경고 팝업 (60°C 이상)
        if max_temp >= 60:
            messagebox.showwarning("온도 경고",
                f"서보 {max_temp_id} ({max_name})의 온도가 {max_temp}°C입니다!\n"
                "과열 위험이 있으니 잠시 휴식을 권장합니다.")

    def update_real_positions_to_sim(self):
        """Real->Sim 모드: 실제 모터 위치를 읽어서 Sim에 반영"""
        # 호출 횟수 카운트
        if not hasattr(self, '_real_to_sim_count'):
            self._real_to_sim_count = 0
        self._real_to_sim_count += 1

        # 디버그: 처음과 매 50번째마다 로그 출력 (100ms * 50 = 5초마다)
        if self._real_to_sim_count == 1 or self._real_to_sim_count % 50 == 0:
            self.log_real(f"[DEBUG] update_real_positions_to_sim 호출 #{self._real_to_sim_count}")
            self.log_real(f"[DEBUG] real_link_to_sim={self.real_link_to_sim}, real_connected={self.real_connected}, sim_connected={self.sim_connected}")

        if not (self.real_link_to_sim and self.real_connected and self.sim_connected):
            if self._real_to_sim_count <= 3:
                self.log_real(f"[DEBUG] 조건 불만족으로 종료")
            return

        try:
            updated_count = 0
            failed_count = 0

            for servo_id in self.real_active_joints.keys():
                # 실제 모터 위치 읽기
                real_pos = self.read_real_servo_position(servo_id)

                if real_pos is not None:
                    updated_count += 1
                    # Real 내부 값 업데이트
                    self.real_active_joints[servo_id]['current'] = real_pos

                    # Sim 값도 업데이트
                    if servo_id in self.sim_active_joints:
                        self.sim_active_joints[servo_id]['current'] = real_pos

                    # Real->Sim 모드: GUI 슬라이더는 업데이트하지 않음
                    # (사용자가 슬라이더로 제어할 수 있도록 유지)
                    # if servo_id in self.real_sliders:
                    #     self.loading_baseline = True
                    #     self.real_sliders[servo_id].set(real_pos)
                    #     self.loading_baseline = False
                else:
                    failed_count += 1

            # 디버그: 처음 3번은 매번, 이후는 50번마다 출력 (5초마다)
            if self._real_to_sim_count <= 3 or self._real_to_sim_count % 50 == 0:
                self.log_real(f"[Real->Sim #{self._real_to_sim_count}] 읽기: 성공 {updated_count}개, 실패 {failed_count}개")

            # Sim 업데이트
            self.update_sim()

        except Exception as e:
            import traceback
            self.log_real(f"[ERR] Real->Sim 업데이트 실패 (#{self._real_to_sim_count}): {e}")
            self.log_real(f"[ERR] Traceback: {traceback.format_exc()}")

        # 100ms 후 다시 호출 (주기적 업데이트, 초당 10회)
        if self.real_link_to_sim and self.real_connected:
            self.root.after(100, self.update_real_positions_to_sim)

    def update_sim_positions_to_real(self):
        """Sim->Real 모드: Sim 값을 읽어서 실제 모터에 반영"""
        # 호출 횟수 카운트
        if not hasattr(self, '_sim_to_real_count'):
            self._sim_to_real_count = 0
        self._sim_to_real_count += 1

        # 디버그: 처음과 매 50번째마다 로그 출력 (100ms * 50 = 5초마다)
        if self._sim_to_real_count == 1 or self._sim_to_real_count % 50 == 0:
            self.log_real(f"[DEBUG] update_sim_positions_to_real 호출 #{self._sim_to_real_count}")
            self.log_real(f"[DEBUG] sim_link_to_real={self.sim_link_to_real}, real_connected={self.real_connected}, sim_connected={self.sim_connected}")

        if not (self.sim_link_to_real and self.real_connected and self.sim_connected):
            if self._sim_to_real_count <= 3:
                self.log_real(f"[DEBUG] 조건 불만족으로 종료")
            return

        try:
            updated_count = 0

            for servo_id in self.sim_active_joints.keys():
                # Sim 값 읽기
                sim_pos = self.sim_active_joints[servo_id]['current']

                # Real 내부 값 업데이트
                self.real_active_joints[servo_id]['current'] = sim_pos

                # 실제 모터에 명령 전송
                if self.real_time_enabled.get():
                    try:
                        self.send_real_servo_command(servo_id, sim_pos)
                        updated_count += 1
                    except Exception as e:
                        pass  # 에러 무시

            # 디버그: 처음 3번은 매번, 이후는 50번마다 출력 (5초마다)
            if self._sim_to_real_count <= 3 or self._sim_to_real_count % 50 == 0:
                self.log_real(f"[Sim->Real #{self._sim_to_real_count}] 전송: {updated_count}개 모터")

        except Exception as e:
            import traceback
            self.log_real(f"[ERR] Sim->Real 업데이트 실패 (#{self._sim_to_real_count}): {e}")
            self.log_real(f"[ERR] Traceback: {traceback.format_exc()}")

        # 100ms 후 다시 호출 (주기적 업데이트, 초당 10회)
        if self.sim_link_to_real and self.real_connected:
            self.root.after(100, self.update_sim_positions_to_real)

    def apply_real_acceleration(self):
        """모든 Real 서보에 ACC 값 적용"""
        if not self.real_connected:
            messagebox.showwarning("연결 필요", "먼저 로봇에 연결해주세요.")
            return

        try:
            acc_value = self.real_acceleration_value.get()

            if acc_value < 0 or acc_value > 254:
                messagebox.showerror("입력 오류", "ACC 값은 0-254 범위여야 합니다.")
                return

            success_count = 0
            fail_count = 0
            gripper_acc = 20

            for servo_id in self.real_active_joints.keys():
                if servo_id in [41, 31]:
                    target_acc = gripper_acc
                else:
                    target_acc = acc_value

                if self.set_real_servo_acceleration(servo_id, target_acc):
                    success_count += 1
                else:
                    fail_count += 1
                time.sleep(0.01)

            # Sim Velocity와 연동
            sim_velocity = 0.1 + (acc_value / 254.0) * 4.9
            self.sim_velocity_value.set(round(sim_velocity, 2))

            result_msg = f"ACC 설정 완료: {success_count}개 성공"
            if fail_count > 0:
                result_msg += f", {fail_count}개 실패"
            result_msg += f" (일반: {acc_value}, 그리퍼: {gripper_acc})"

            self.log_real(result_msg)
            messagebox.showinfo("ACC 설정", result_msg)

        except ValueError:
            messagebox.showerror("입력 오류", "유효한 숫자를 입력해주세요.")
        except Exception as e:
            self.log_real(f"ACC 설정 오류: {e}")
            messagebox.showerror("오류", f"ACC 설정 중 오류 발생: {e}")

    def on_sim_velocity_change(self, *args):
        """Sim Velocity 변경 시 Real ACC 자동 연동"""
        try:
            velocity = self.sim_velocity_value.get()

            if velocity < 0.1 or velocity > 5.0:
                return

            real_acc = int(((velocity - 0.1) / 4.9) * 254.0)
            real_acc = max(0, min(254, real_acc))

            self.real_acceleration_value.set(real_acc)
            self.log_real(f"[연동] ACC <- {real_acc} (from Velocity {velocity:.2f} rad/s)")

        except Exception as e:
            pass

    # ===================================================================
    # ROS 연동 로직
    # ===================================================================

    def connect_ros(self):
        """ROS2 연동 - 라이다 + 로봇 시각화 시작/중지 토글"""
        try:
            # 이미 실행 중이면 중지
            if hasattr(self, 'ros_process') and self.ros_process and self.ros_process.poll() is None:
                self.log_real("ROS2 중지 중...")
                import signal

                # 1. 먼저 ROS 프로세스 강제 종료 (라이다 포트 해제)
                self._kill_ros_processes()
                time.sleep(0.5)

                try:
                    # 프로세스 그룹 전체 종료
                    os.killpg(os.getpgid(self.ros_process.pid), signal.SIGKILL)
                except:
                    pass
                self.ros_process = None
                if hasattr(self, 'rviz_process'):
                    self.rviz_process = None

                # 2. 라이다 모터 정지 (포트가 해제된 후)
                time.sleep(0.5)
                self._stop_lidar_motor()

                # Sim 연결 해제
                self.sim_connected = False

                self.ros_status.config(text="[ ] ROS 꺼짐", foreground="gray")
                self.ros_connect_btn.config(text="ROS 연동")
                self.log_real("[OK] ROS2 중지됨")
                return

            self.log_real("ROS2 시작 중... (라이다 + 로봇)")

            # ROS2 프로세스 시작 (라이다 + 로봇 시각화)
            threading.Thread(target=self._start_ros_full, daemon=True).start()

        except Exception as e:
            messagebox.showerror("ROS 연결 오류", f"ROS 시작 실패:\n{e}")
            self.log_real(f"ROS 시작 실패: {e}")

    def _stop_lidar_motor(self):
        """라이다 모터 정지 (pyrplidar 사용)"""
        try:
            import glob

            # 사용 가능한 포트 찾기
            ports = glob.glob('/dev/ttyUSB*')
            if not ports:
                self.log_real("[WARN] 라이다 포트 없음")
                return
            port = ports[0]

            # pyrplidar로 모터 정지 (가장 확실한 방법)
            try:
                from pyrplidar import PyRPlidar
                lidar = PyRPlidar()
                lidar.connect(port=port, baudrate=256000, timeout=3)
                time.sleep(1)
                lidar.stop()
                time.sleep(0.5)
                lidar.set_motor_pwm(0)
                time.sleep(1)
                lidar.disconnect()
                self.log_real("[OK] 라이다 모터 정지 (pyrplidar)")
                return
            except Exception as e:
                self.log_real(f"[WARN] pyrplidar 정지 실패: {e}")

            # 백업: serial로 시도
            try:
                import serial
                ser = serial.Serial(port, 256000, timeout=0.5)
                ser.write(b'\xa5\x25')
                ser.setDTR(True)
                time.sleep(0.3)
                ser.close()
            except:
                pass

            self.log_real("[OK] 라이다 모터 정지")
        except Exception as e:
            self.log_real(f"[WARN] 라이다 모터 정지 실패: {e}")

    def _reset_lidar_usb(self):
        """USB 리셋으로 라이다 모터 강제 정지"""
        import fcntl
        import glob

        USBDEVFS_RESET = 21780

        # CP210x 장치 찾기 (라이다)
        for usb_dev in glob.glob('/dev/bus/usb/*/*'):
            try:
                # 장치 열어서 리셋
                fd = os.open(usb_dev, os.O_WRONLY)
                # 일단 시도
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                os.close(fd)
            except:
                continue

    def _kill_ros_processes(self):
        """남아있는 ROS 프로세스 강제 종료"""
        try:
            import subprocess
            # sllidar, rviz, ros2 launch 프로세스 종료
            subprocess.run(['pkill', '-9', '-f', 'sllidar'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'rviz2'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'ros2 launch'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], capture_output=True)
            subprocess.run(['pkill', '-9', '-f', 'joint_state_publisher'], capture_output=True)
            self.log_real("[OK] ROS 프로세스 정리 완료")
        except Exception as e:
            self.log_real(f"[WARN] ROS 프로세스 정리 실패: {e}")

    def _start_ros_full(self):
        """ROS2 로봇 시각화 시작 (라이다 자동 감지)"""
        try:
            # 이미 실행 중이면 다시 시작하지 않음
            if hasattr(self, 'ros_process') and self.ros_process and self.ros_process.poll() is None:
                self.log_real("[WARN] ROS가 이미 실행 중입니다.")
                return

            # 라이다 연결 확인
            import glob
            lidar_connected = bool(glob.glob('/dev/ttyUSB*'))

            if lidar_connected:
                self.log_real("ROS2 시작 중... (라이다 + 로봇)")
                launch_file = os.path.join(_GUI_ROOT, "ros_files", "rx1_with_lidar.launch.py")
            else:
                self.log_real("ROS2 시작 중... (로봇만 - 라이다 없음)")
                launch_file = os.path.join(_GUI_ROOT, "ros_files", "rx1_robot_only.launch.py")

            # ========== 디버그: 로드되는 파일 정보 ==========
            base_dir = _GUI_ROOT
            urdf_file = f"{base_dir}/urdf/RRR_Cater/RX1/combined/rx1_with_lidar.urdf"
            rviz_config = f"{base_dir}/urdf/RRR_Cater/RX1/rx1_description/rviz/urdf.rviz"
            mesh_dir = f"{base_dir}/urdf/RRR_Cater/RX1/combined"
            mesh_dir_desc = f"{base_dir}/urdf/RRR_Cater/RX1/rx1_description/meshes"

            self.log_real("=" * 50)
            self.log_real("[DEBUG] ROS 파일 정보:")
            self.log_real(f"  Launch: {launch_file}")
            self.log_real(f"  URDF: {urdf_file}")
            self.log_real(f"  RViz: {rviz_config}")
            self.log_real(f"  Lidar: {'연결됨' if lidar_connected else '없음'}")
            self.log_real("-" * 50)
            self.log_real("[DEBUG] 메시 파일 (combined/):")

            # combined 폴더 메시 파일 목록
            import os
            if os.path.exists(mesh_dir):
                obj_files = [f for f in os.listdir(mesh_dir) if f.endswith('.obj')]
                stl_files = [f for f in os.listdir(mesh_dir) if f.endswith('.stl')]
                self.log_real(f"  OBJ: {len(obj_files)}개")
                self.log_real(f"  STL: {len(stl_files)}개")

            self.log_real("[DEBUG] 메시 파일 (rx1_description/meshes/):")
            if os.path.exists(mesh_dir_desc):
                for subdir in ['visual', 'collision']:
                    subpath = os.path.join(mesh_dir_desc, subdir)
                    if os.path.exists(subpath):
                        files = os.listdir(subpath)
                        stl_count = len([f for f in files if f.endswith('.stl')])
                        obj_count = len([f for f in files if f.endswith('.obj')])
                        self.log_real(f"  {subdir}/: STL={stl_count}, OBJ={obj_count}")
            self.log_real("=" * 50)

            # ROS2 launch 스크립트 생성 (터미널 실행과 동일하게)
            ros2_ws_setup = os.path.join(_PROJECT_ROOT, "ros2_ws", "install", "setup.bash")
            ros_script = f'''#!/bin/bash
source /opt/ros/humble/setup.bash
if [ -f "{ros2_ws_setup}" ]; then
    source "{ros2_ws_setup}"
fi
ros2 launch {launch_file} >> /tmp/ros_launch_v7.log 2>&1
'''
            script_path = "/tmp/ros_launch_v7.sh"
            with open(script_path, 'w') as f:
                f.write(ros_script)
            os.chmod(script_path, 0o755)

            # 로그 파일 초기화
            with open('/tmp/ros_launch_v7.log', 'w') as f:
                f.write(f"ROS2 Launch started at {time.strftime('%H:%M:%S')}\n")

            # 스크립트 직접 실행 (터미널과 동일한 방식)
            self.ros_process = subprocess.Popen(
                [script_path],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            self.log_real("[OK] ROS2 Launch 실행됨")

            # RViz2 스크립트 생성 (Qt 플러그인 충돌 방지)
            rviz_config = os.path.join(_GUI_ROOT, "ros_files", "rx1_lidar_config.rviz")
            rviz_script = f'''#!/bin/bash
# OpenCV Qt 플러그인 충돌 방지
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH
export QT_QPA_PLATFORM=xcb

source /opt/ros/humble/setup.bash
if [ -f "{ros2_ws_setup}" ]; then
    source "{ros2_ws_setup}"
fi
sleep 3
rviz2 -d {rviz_config} >> /tmp/rviz2_log.txt 2>&1
'''
            rviz_script_path = "/tmp/rviz_launch_v7.sh"
            with open(rviz_script_path, 'w') as f:
                f.write(rviz_script)
            os.chmod(rviz_script_path, 0o755)

            # RViz 스크립트 실행
            self.rviz_process = subprocess.Popen(
                [rviz_script_path],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            self.log_real("[OK] RViz2 시작 예약됨 (3초 후)")

            if lidar_connected:
                self.log_real("[OK] ROS2 시작됨 (라이다 + 로봇)")
            else:
                self.log_real("[OK] ROS2 시작됨 (로봇만)")

            # Sim 연결 상태 설정 (ROS가 시작되면 Sim도 연결된 것으로 처리)
            self.sim_connected = True

            self.ros_status.config(text="[O] ROS 켜짐", foreground="green")
            self.ros_connect_btn.config(text="ROS 중지")

            # 로그 모니터링
            threading.Thread(target=self._monitor_ros_log, daemon=True).start()

        except Exception as e:
            self.log_real(f"[ERR] ROS 시작 실패: {e}")
            self.ros_status.config(text="[X] ROS 오류", foreground="red")

    def connect_sim(self):
        """Sim 연결 (레거시 - 사용안함)"""
        pass

    def _start_ros_lidar(self):
        """ROS2 라이다 노드 시작"""
        try:
            # 이미 실행 중이면 다시 시작하지 않음
            if hasattr(self, 'ros_process') and self.ros_process and self.ros_process.poll() is None:
                self.log_real("[WARN] ROS 라이다가 이미 실행 중입니다.")
                return

            self.log_real("ROS2 라이다 노드 시작 중...")

            # ROS2 launch 명령어 (A2M12용)
            # source 명령어와 ros2 launch를 함께 실행
            cmd = '''
            source /opt/ros/humble/setup.bash && \
            source /opt/ros/humble/setup.bash && \
            ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
            '''

            self.ros_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            self.log_real("[OK] ROS2 라이다 + RViz 시작됨")
            self.ros_status.config(text="[O] ROS 켜짐", foreground="green")
            self.ros_connect_btn.config(text="ROS 중지")

            # 로그 모니터링
            threading.Thread(target=self._monitor_ros_log, daemon=True).start()

        except Exception as e:
            self.log_real(f"[ERR] ROS 라이다 시작 실패: {e}")
            self.ros_status.config(text="[X] ROS 오류", foreground="red")

    def _monitor_ros_log(self):
        """ROS 로그 모니터링 (파일 기반) + 프로세스 종료 감지"""
        log_file = '/tmp/ros_launch_v7.log'
        last_pos = 0
        try:
            # 잠시 대기 후 로그 확인 시작
            time.sleep(3)
            while self.ros_process and self.ros_process.poll() is None:
                try:
                    with open(log_file, 'r') as f:
                        f.seek(last_pos)
                        new_lines = f.readlines()
                        last_pos = f.tell()
                        for line in new_lines:
                            line = line.strip()
                            if line and any(keyword in line.lower() for keyword in ['error', 'warn', 'segment', 'connected', 'started']):
                                self.root.after(0, lambda msg=line: self.log_real(f"[ROS] {msg}"))
                except FileNotFoundError:
                    pass
                time.sleep(1)

            # ROS 프로세스가 종료됨
            self.root.after(0, self._on_ros_process_ended)
        except:
            pass

    def _on_ros_process_ended(self):
        """ROS 프로세스가 종료되었을 때 호출"""
        try:
            self.log_real("[INFO] ROS 프로세스 종료 감지 - 자동 정리 중...")

            # 라이다 모터 정지
            self._stop_lidar_motor()

            # 남은 ROS 프로세스 정리
            self._kill_ros_processes()

            # UI 상태 업데이트
            self.ros_status.config(text="[X] ROS 꺼짐", foreground="red")
            self.ros_connect_btn.config(text="ROS 연동")
            self.ros_process = None
            if hasattr(self, 'rviz_process'):
                self.rviz_process = None

            self.log_real("[OK] ROS 자동 정리 완료")
        except Exception as e:
            self.log_real(f"[WARN] ROS 자동 정리 중 오류: {e}")

    # ========== OpenXR (WiVRn) 텔레옵 ==========
    def set_vr_arm_select(self, arm):
        """VR 팔 선택 (all, left, right)"""
        self.vr_arm_select.set(arm)
        # 버튼 색상 업데이트
        active_color = '#00b894'
        inactive_color = '#636e72'
        self.vr_arm_all_btn.config(bg=active_color if arm == "all" else inactive_color)
        self.vr_arm_left_btn.config(bg=active_color if arm == "left" else inactive_color)
        self.vr_arm_right_btn.config(bg=active_color if arm == "right" else inactive_color)
        self.log_real(f"[VR] 팔 선택: {arm.upper()}")

    def toggle_vr_teleop(self):
        """VR 텔레옵 토글"""
        if self.vr_enabled:
            self.stop_vr_teleop()
        else:
            self.start_vr_teleop()

    def start_vr_teleop(self):
        """VR 텔레옵 시작 (OpenXR/WiVRn)"""
        # 연결 대기 취소 플래그 초기화
        self._vr_cancel_wait = False

        # 기존 OpenXR 인스턴스 정리 (중복 방지)
        if hasattr(self, 'xr_instance') and self.xr_instance:
            self.log_real("[INFO] 기존 OpenXR 세션 정리 중...")
            self._cleanup_openxr()
            self.xr_instance = None
            self.xr_session = None

        # WiVRn 자동 실행
        import subprocess
        try:
            result = subprocess.run(['pgrep', '-f', 'wivrn'], capture_output=True)
            if result.returncode != 0:
                self.log_real("[VR] WiVRn 자동 시작 중...")
                self.vr_teleop_status.config(text="[WiVRn 시작 중]", foreground="orange")
                self.root.update()
                # WiVRn 백그라운드 실행
                subprocess.Popen(['flatpak', 'run', 'io.github.wivrn.wivrn'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.log_real("[OK] WiVRn 시작됨")
                # WiVRn 초기화 대기
                import time
                time.sleep(2)
            else:
                self.log_real("[OK] WiVRn 이미 실행 중")
        except Exception as e:
            self.log_real(f"[WARN] WiVRn 자동 실행 실패: {e}")
            self.log_real("       수동 실행: flatpak run io.github.wivrn.wivrn")

        # VR 연결 대기 모드
        self.vr_teleop_status.config(text="[VR 연결 대기]", foreground="orange")
        self.log_real("[VR] Quest 연결 대기 중...")
        self.log_real("     Quest에서 WiVRn 앱을 실행하고 연결하세요")
        self.root.update()

        # OpenXR 확인
        try:
            import xr
            self.log_real("[OK] OpenXR 라이브러리 로드됨")
        except ImportError as e:
            self.log_real(f"[ERR] OpenXR 없음: {e}")
            messagebox.showerror("VR 텔레옵",
                "OpenXR 라이브러리가 필요합니다.\n\n"
                "설치: pip install pyopenxr")
            return

        # OpenXR 초기화 (연결 대기 모드)
        import time
        max_retries = 30  # 최대 30초 대기
        connected = False

        for retry in range(max_retries):
            try:
                self._init_openxr()
                self.log_real("[OK] OpenXR/WiVRn 연결됨")
                connected = True
                break
            except Exception as e:
                # 연결 대기 중 상태 표시
                dots = "." * ((retry % 3) + 1)
                self.vr_teleop_status.config(text=f"[VR 연결 대기{dots}]", foreground="orange")
                self.root.update()

                if retry == 0:
                    self.log_real(f"[VR] OpenXR 연결 대기 중... (Quest에서 WiVRn 연결)")

                time.sleep(1)

                # 사용자가 VR 중지 버튼을 눌렀는지 확인
                if hasattr(self, '_vr_cancel_wait') and self._vr_cancel_wait:
                    self._vr_cancel_wait = False
                    self.log_real("[VR] 연결 대기 취소됨")
                    self.vr_teleop_status.config(text="[VR 꺼짐]", foreground="gray")
                    return

        if not connected:
            self.log_real("[ERR] OpenXR 연결 시간 초과 (30초)")
            self.vr_teleop_status.config(text="[VR 연결 실패]", foreground="red")
            messagebox.showerror("VR 텔레옵",
                "OpenXR 연결 시간 초과\n\n"
                "1. WiVRn이 실행 중인지 확인\n"
                "2. Quest에서 WiVRn 앱 실행\n"
                "3. 같은 WiFi 네트워크 확인")
            return

        # IK 활성화 (항상 robot baseline 기준으로 초기화)
        if not self.ik_enabled.get():
            self.ik_enabled.set(True)
            # IK는 현재 active baseline 기준으로 초기화
            self.init_ik_solver()
            baseline_name = "VR" if self.active_baseline_type == 'vr' else "Robot"
            self.log_real(f"[INFO] IK 자동 활성화됨 ({baseline_name} baseline 기준)")

        # 로봇을 기본자세로 이동 (VR baseline이면 VR 위치 유지)
        self._move_to_baseline_for_vr()

        self.vr_enabled = True
        self.vr_calibrated = {'right': False, 'left': False}
        self.vr_base_pose = {'right': None, 'left': None}
        self.vr_base_orientation = {'right': None, 'left': None}
        self.vr_rotation_offset = {'right': [0.0, 0.0, 0.0], 'left': [0.0, 0.0, 0.0]}

        # VR 데이터 수신 스레드 시작
        self.vr_thread = threading.Thread(target=self._vr_openxr_receiver, daemon=True)
        self.vr_thread.start()

        self.vr_teleop_btn.config(text="VR 중지")
        self.vr_teleop_status.config(text="[VR 대기]", foreground="orange")
        self.log_real("[OK] OpenXR 수신 대기 중")
        self.log_real("[INFO] 로봇이 기본자세로 이동했습니다")
        self.log_real("     사람도 동일하게 포즈를 잡고")
        self.log_real("     양쪽 트리거를 동시에 눌러 시작하세요")

    def _move_to_baseline_for_vr(self):
        """VR 텔레옵을 위해 로봇을 기본자세로 이동"""
        # 활성 baseline 선택 (VR baseline이면 vr_baseline 사용)
        if self.active_baseline_type == 'vr':
            active_baseline = self.vr_baseline
            self.log_real("[VR] VR 기본자세로 이동 중...")
        else:
            active_baseline = self.robot_baseline
            self.log_real("[VR] 로봇 기본자세로 이동 중...")

        # IK 콜백 방지 플래그 설정
        self._ik_updating = True

        try:
            # 모든 팔 관절을 baseline으로 설정
            arm_servo_ids = [11, 12, 13, 14, 15, 16, 17,  # 오른팔
                             21, 22, 23, 24, 25, 26, 27]  # 왼팔

            for sid in arm_servo_ids:
                baseline_val = active_baseline.get(sid, 2048)
                if sid in self.sim_active_joints:
                    self.sim_active_joints[sid]['current'] = baseline_val
                if sid in self.real_active_joints:
                    self.real_active_joints[sid]['current'] = baseline_val
                if sid in self.real_sliders:
                    self.real_sliders[sid].set(baseline_val)

            # 그리퍼도 baseline으로 (열림)
            for gid in [41, 31]:
                baseline_val = active_baseline.get(gid, 0)
                if gid in self.sim_active_joints:
                    self.sim_active_joints[gid]['current'] = baseline_val
                if gid in self.real_active_joints:
                    self.real_active_joints[gid]['current'] = baseline_val
                if gid in self.real_sliders:
                    self.real_sliders[gid].set(baseline_val)

            # IK solver가 없으면 현재 active baseline 기준으로 초기화
            if self.ik_solver.get('right') is None:
                self.init_ik_solver()
                self.ik_enabled.set(True)
                baseline_name = "VR" if self.active_baseline_type == 'vr' else "Robot"
                self.log_real(f"[VR] IK solver 초기화됨 ({baseline_name} baseline 기준)")

            # IK 슬라이더 0으로 리셋 (현재 baseline 위치가 기준점)
            for arm in ['right', 'left']:
                if arm in self.ik_sliders:
                    for axis in ['x', 'y', 'z']:
                        if axis in self.ik_sliders[arm]:
                            self.ik_sliders[arm][axis].set(0)
                            self.ik_values[arm][axis] = 0
                            if axis in self.ik_value_labels[arm]:
                                self.ik_value_labels[arm][axis].config(text="0.0cm")

        finally:
            # IK 콜백 방지 플래그 해제
            self._ik_updating = False

        # 시뮬레이션 업데이트
        self.schedule_sim_update()

    def _init_openxr(self):
        """OpenXR 인스턴스 및 세션 초기화 (GLFW + OpenGL)"""
        import xr
        import glfw
        from OpenGL import GL
        import ctypes

        # GLFW 초기화
        if not glfw.init():
            raise RuntimeError("GLFW 초기화 실패")

        # OpenGL 컨텍스트 힌트 설정
        glfw.window_hint(glfw.DOUBLEBUFFER, True)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 5)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)  # 숨겨진 창

        # 창 생성
        self._glfw_window = glfw.create_window(640, 480, "XR", None, None)
        if not self._glfw_window:
            glfw.terminate()
            raise RuntimeError("GLFW 창 생성 실패")

        glfw.make_context_current(self._glfw_window)
        self.log_real("[XR] GLFW/OpenGL 컨텍스트 생성됨")

        # 사용 가능한 확장 확인
        available_extensions = xr.enumerate_instance_extension_properties()
        # extension_name이 bytes일 수 있으므로 decode
        available_ext_names = []
        for ext in available_extensions:
            name = ext.extension_name
            if isinstance(name, bytes):
                name = name.decode('utf-8')
            available_ext_names.append(name)
        self.log_real(f"[XR] 사용 가능한 확장: {len(available_ext_names)}개")

        # OpenGL 확장 활성화
        enabled_extensions = []
        if "XR_KHR_opengl_enable" in available_ext_names:
            enabled_extensions.append("XR_KHR_opengl_enable")
            self.log_real("[XR] OpenGL 확장 활성화")
        else:
            self.log_real("[XR] 경고: XR_KHR_opengl_enable 확장 없음!")

        # OpenXR 인스턴스 생성
        self.xr_instance = xr.create_instance(
            create_info=xr.InstanceCreateInfo(
                application_info=xr.ApplicationInfo(
                    application_name="RRR_Robot_Teleop",
                    application_version=1,
                    engine_name="RRR",
                    engine_version=1,
                    api_version=xr.Version(1, 0, 0)
                ),
                enabled_extension_names=enabled_extensions,
            )
        )
        self.log_real(f"[XR] 인스턴스 생성됨")

        # 시스템 가져오기
        self.xr_system_id = xr.get_system(
            instance=self.xr_instance,
            get_info=xr.SystemGetInfo(
                form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY
            )
        )
        self.log_real(f"[XR] 시스템 ID: {self.xr_system_id}")

        # OpenGL 그래픽 바인딩 생성
        import ctypes
        from ctypes import c_void_p, c_uint32, c_ulong, pointer, byref, addressof

        # X11 디스플레이 정보 가져오기
        x_display = glfw.get_x11_display()  # Returns integer (pointer address)
        glx_context = glfw.get_glx_context(self._glfw_window)  # Returns integer (pointer address)
        glx_drawable = glfw.get_x11_window(self._glfw_window)  # Returns integer (XID)

        self.log_real(f"[XR] X11 Display: {x_display}")
        self.log_real(f"[XR] GLX Context: {glx_context}")
        self.log_real(f"[XR] GLX Drawable: {glx_drawable}")

        # visualid 가져오기 - X11에서 현재 visual 정보
        try:
            from OpenGL.GLX import glXGetCurrentDrawable, glXGetCurrentContext
            from OpenGL.raw.GLX._types import Display
            import OpenGL.GL as gl
            # 기본 visual ID 사용 (0은 런타임이 선택하도록 함)
            visualid = 0
        except Exception as e:
            self.log_real(f"[XR] Visual ID 가져오기 실패, 기본값 사용: {e}")
            visualid = 0

        # OpenXR 그래픽 요구사항 확인 - 세션 생성 전에 반드시 호출해야 함!
        graphics_requirements = xr.get_opengl_graphics_requirements_khr(
            self.xr_instance, self.xr_system_id
        )
        self.log_real(f"[XR] OpenGL 요구사항: min={graphics_requirements.min_api_version_supported}, max={graphics_requirements.max_api_version_supported}")

        # GraphicsBindingOpenGLXlibKHR 구조체 수동 생성
        # pyopenxr는 OpenGL 타입을 사용하므로 POINTER + ctypes.cast 필요
        from OpenGL.raw.GLX._types import (
            struct__XDisplay,
            struct___GLXFBConfigRec,
            struct___GLXcontextRec
        )
        from ctypes import POINTER as CPOINTER
        LP_Display = CPOINTER(struct__XDisplay)
        LP_GLXFBConfig = CPOINTER(struct___GLXFBConfigRec)
        LP_GLXContext = CPOINTER(struct___GLXcontextRec)

        graphics_binding = xr.GraphicsBindingOpenGLXlibKHR()
        # type 필드는 자동 설정됨
        # 포인터 타입으로 캐스팅
        graphics_binding.x_display = ctypes.cast(x_display, LP_Display)
        graphics_binding.visualid = visualid  # uint32
        graphics_binding.glx_fbconfig = ctypes.cast(0, LP_GLXFBConfig)  # NULL
        graphics_binding.glx_drawable = glx_drawable  # XID (unsigned long)
        graphics_binding.glx_context = ctypes.cast(glx_context, LP_GLXContext)

        self.log_real(f"[XR] OpenGL 바인딩 생성됨 (type={graphics_binding.type})")
        self._graphics_binding = graphics_binding  # GC 방지를 위해 참조 유지

        # next 포인터 준비 (pyopenxr는 _next 필드를 c_void_p로 직접 설정해야 함)
        binding_ptr = ctypes.pointer(graphics_binding)
        binding_void_ptr = ctypes.cast(binding_ptr, c_void_p)

        # 세션 생성 (OpenGL 바인딩 포함)
        session_create_info = xr.SessionCreateInfo(system_id=self.xr_system_id)
        session_create_info._next = binding_void_ptr

        self.xr_session = xr.create_session(
            instance=self.xr_instance,
            create_info=session_create_info
        )
        self.log_real(f"[XR] 세션 생성됨")

        # 레퍼런스 스페이스 생성 (LOCAL)
        space_create_info = xr.ReferenceSpaceCreateInfo(
            reference_space_type=xr.ReferenceSpaceType.LOCAL,
            pose_in_reference_space=xr.Posef(
                orientation=xr.Quaternionf(0, 0, 0, 1),
                position=xr.Vector3f(0, 0, 0)
            )
        )
        self.xr_space = xr.create_reference_space(self.xr_session, space_create_info)

        # VIEW 레퍼런스 스페이스 생성 (헤드셋 위치 추적용)
        view_space_info = xr.ReferenceSpaceCreateInfo(
            reference_space_type=xr.ReferenceSpaceType.VIEW,
            pose_in_reference_space=xr.Posef(
                orientation=xr.Quaternionf(0, 0, 0, 1),
                position=xr.Vector3f(0, 0, 0)
            )
        )
        self.xr_view_space = xr.create_reference_space(self.xr_session, view_space_info)

        # 뷰 설정 가져오기 (스왑체인용)
        self._setup_swapchains()

        # 액션 세트 생성
        self._create_xr_actions()

        # VR 스레드에서 컨텍스트를 사용할 수 있도록 해제
        glfw.make_context_current(None)
        self.log_real("[XR] OpenGL 컨텍스트 해제됨 (VR 스레드용)")

    def _setup_swapchains(self):
        """스왑체인 설정 (WiVRn 스트리밍용)"""
        import xr
        from OpenGL import GL

        # 뷰 설정 가져오기
        view_configs = xr.enumerate_view_configuration_views(
            self.xr_instance,
            self.xr_system_id,
            xr.ViewConfigurationType.PRIMARY_STEREO
        )
        self.xr_views = view_configs
        self.log_real(f"[XR] 뷰 수: {len(view_configs)}")

        # 스왑체인 포맷 확인
        swapchain_formats = xr.enumerate_swapchain_formats(self.xr_session)
        # GL_SRGB8_ALPHA8 (0x8C43) 또는 GL_RGBA8 (0x8058) 선택
        chosen_format = GL.GL_RGBA8
        for fmt in swapchain_formats:
            if fmt == 0x8C43:  # GL_SRGB8_ALPHA8
                chosen_format = fmt
                break
            elif fmt == GL.GL_RGBA8:
                chosen_format = fmt

        self.log_real(f"[XR] 스왑체인 포맷: {hex(chosen_format)}")

        # 각 눈에 대한 스왑체인 생성
        self.xr_swapchains = []
        self.xr_swapchain_images = []

        for i, view_config in enumerate(view_configs):
            width = view_config.recommended_image_rect_width
            height = view_config.recommended_image_rect_height

            swapchain_create_info = xr.SwapchainCreateInfo(
                usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT | xr.SwapchainUsageFlags.SAMPLED_BIT,
                format=chosen_format,
                sample_count=1,
                width=width,
                height=height,
                face_count=1,
                array_size=1,
                mip_count=1
            )

            swapchain = xr.create_swapchain(self.xr_session, swapchain_create_info)
            self.xr_swapchains.append(swapchain)

            # 스왑체인 이미지 가져오기
            images = xr.enumerate_swapchain_images(swapchain, xr.SwapchainImageOpenGLKHR)
            self.xr_swapchain_images.append(images)

            self.log_real(f"[XR] 스왑체인 {i}: {width}x{height}, 이미지 {len(images)}개")

        # 프로젝션 뷰 초기화
        self.xr_projection_views = []
        for i, view_config in enumerate(view_configs):
            proj_view = xr.CompositionLayerProjectionView(
                sub_image=xr.SwapchainSubImage(
                    swapchain=self.xr_swapchains[i],
                    image_rect=xr.Rect2Di(
                        offset=xr.Offset2Di(0, 0),
                        extent=xr.Extent2Di(
                            view_config.recommended_image_rect_width,
                            view_config.recommended_image_rect_height
                        )
                    ),
                    image_array_index=0
                )
            )
            self.xr_projection_views.append(proj_view)

    def _create_xr_actions(self):
        """OpenXR 액션 (컨트롤러 입력) 생성"""
        import xr

        # 액션 세트 생성
        action_set_info = xr.ActionSetCreateInfo(
            action_set_name="teleop",
            localized_action_set_name="Teleop Controls",
            priority=0
        )
        self.xr_action_set = xr.create_action_set(self.xr_instance, action_set_info)

        # 컨트롤러 경로
        self.xr_hand_paths = {
            'left': xr.string_to_path(self.xr_instance, "/user/hand/left"),
            'right': xr.string_to_path(self.xr_instance, "/user/hand/right")
        }

        # 포즈 액션 (컨트롤러 위치)
        pose_action_info = xr.ActionCreateInfo(
            action_name="hand_pose",
            action_type=xr.ActionType.POSE_INPUT,
            count_subaction_paths=2,
            subaction_paths=[self.xr_hand_paths['left'], self.xr_hand_paths['right']],
            localized_action_name="Hand Pose"
        )
        self.xr_pose_action = xr.create_action(self.xr_action_set, pose_action_info)

        # 그립 액션 (아날로그 - squeeze/value 사용)
        grip_action_info = xr.ActionCreateInfo(
            action_name="grip",
            action_type=xr.ActionType.FLOAT_INPUT,
            count_subaction_paths=2,
            subaction_paths=[self.xr_hand_paths['left'], self.xr_hand_paths['right']],
            localized_action_name="Grip"
        )
        self.xr_grip_action = xr.create_action(self.xr_action_set, grip_action_info)

        # 트리거 액션 (아날로그)
        trigger_action_info = xr.ActionCreateInfo(
            action_name="trigger",
            action_type=xr.ActionType.FLOAT_INPUT,
            count_subaction_paths=2,
            subaction_paths=[self.xr_hand_paths['left'], self.xr_hand_paths['right']],
            localized_action_name="Trigger"
        )
        self.xr_trigger_action = xr.create_action(self.xr_action_set, trigger_action_info)

        # 액션 스페이스 생성 (포즈용)
        self.xr_hand_spaces = {}
        for hand, path in self.xr_hand_paths.items():
            space_info = xr.ActionSpaceCreateInfo(
                action=self.xr_pose_action,
                subaction_path=path,
                pose_in_action_space=xr.Posef(
                    orientation=xr.Quaternionf(0, 0, 0, 1),
                    position=xr.Vector3f(0, 0, 0)
                )
            )
            self.xr_hand_spaces[hand] = xr.create_action_space(self.xr_session, space_info)

        # 바인딩 제안 (Quest 컨트롤러)
        self._suggest_xr_bindings()

        # 액션 세트 연결
        attach_info = xr.SessionActionSetsAttachInfo(
            action_sets=[self.xr_action_set]
        )
        xr.attach_session_action_sets(self.xr_session, attach_info)

    def _suggest_xr_bindings(self):
        """컨트롤러 바인딩 제안"""
        import xr

        # Quest Touch 컨트롤러 프로파일
        profile_path = xr.string_to_path(self.xr_instance, "/interaction_profiles/oculus/touch_controller")

        bindings = [
            # 왼손 포즈
            xr.ActionSuggestedBinding(
                action=self.xr_pose_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/left/input/grip/pose")
            ),
            # 오른손 포즈
            xr.ActionSuggestedBinding(
                action=self.xr_pose_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/right/input/grip/pose")
            ),
            # 왼손 그립
            xr.ActionSuggestedBinding(
                action=self.xr_grip_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/left/input/squeeze/value")
            ),
            # 오른손 그립
            xr.ActionSuggestedBinding(
                action=self.xr_grip_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/right/input/squeeze/value")
            ),
            # 왼손 트리거
            xr.ActionSuggestedBinding(
                action=self.xr_trigger_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/left/input/trigger/value")
            ),
            # 오른손 트리거
            xr.ActionSuggestedBinding(
                action=self.xr_trigger_action,
                binding=xr.string_to_path(self.xr_instance, "/user/hand/right/input/trigger/value")
            ),
        ]

        suggest_info = xr.InteractionProfileSuggestedBinding(
            interaction_profile=profile_path,
            suggested_bindings=bindings
        )

        try:
            xr.suggest_interaction_profile_bindings(self.xr_instance, suggest_info)
            self.log_real("[XR] Quest 컨트롤러 바인딩 완료")
        except Exception as e:
            self.log_real(f"[XR] 바인딩 경고: {e}")

    def _vr_openxr_receiver(self):
        """OpenXR에서 VR 데이터 수신"""
        import time
        import xr
        import ctypes
        import glfw

        # VR 디버그 로그 파일
        vr_debug_file = open("vr_debug.txt", "w")
        def vr_log(msg):
            timestamp = time.strftime("%H:%M:%S")
            vr_debug_file.write(f"[{timestamp}] {msg}\n")
            vr_debug_file.flush()

        vr_log("OpenXR 수신 스레드 시작")

        # OpenGL 컨텍스트를 이 스레드에서 활성화해야 함!
        try:
            glfw.make_context_current(self._glfw_window)
            vr_log("OpenGL 컨텍스트 활성화됨 (VR 스레드)")
        except Exception as e:
            vr_log(f"OpenGL 컨텍스트 활성화 실패: {e}")

        self.root.after(0, lambda: self.log_real("[VR] OpenXR 수신 스레드 시작 (로그: vr_debug.txt)"))

        session_running = False
        session_state = xr.SessionState.UNKNOWN
        frame_count = 0

        connected = False
        vr_log("메인 루프 시작")
        while self.vr_enabled:
            try:
                # 이벤트 폴링 및 세션 상태 처리
                try:
                    while True:
                        event = xr.poll_event(self.xr_instance)
                        vr_log(f"이벤트 수신: type={event.type}")

                        # 세션 상태 변경 이벤트 확인 (type 필드로 판별)
                        if event.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                            # EventDataSessionStateChanged로 캐스팅
                            session_event = ctypes.cast(
                                ctypes.byref(event),
                                ctypes.POINTER(xr.EventDataSessionStateChanged)
                            ).contents
                            session_state = session_event.state
                            vr_log(f"세션 상태 변경: {session_state}")
                            self.root.after(0, lambda s=session_state: self.log_real(f"[XR] 세션 상태: {s}"))

                            if session_state == xr.SessionState.READY:
                                # 세션 시작
                                begin_info = xr.SessionBeginInfo(
                                    primary_view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO
                                )
                                xr.begin_session(self.xr_session, begin_info)
                                session_running = True
                                self.root.after(0, lambda: self.log_real("[XR] 세션 시작됨 - Quest 착용 대기 중..."))
                                self.root.after(0, lambda: self.vr_teleop_status.config(text="[Quest 착용!]", foreground="orange"))

                            elif session_state == xr.SessionState.SYNCHRONIZED:
                                self.root.after(0, lambda: self.log_real("[XR] SYNCHRONIZED - Quest 착용 대기"))
                                self.root.after(0, lambda: self.vr_teleop_status.config(text="[Quest 착용!]", foreground="orange"))

                            elif session_state == xr.SessionState.VISIBLE:
                                self.root.after(0, lambda: self.log_real("[XR] VISIBLE - 포커스 대기"))
                                self.root.after(0, lambda: self.vr_teleop_status.config(text="[VR 준비]", foreground="yellow"))

                            elif session_state == xr.SessionState.FOCUSED:
                                self.root.after(0, lambda: self.log_real("[XR] 세션 FOCUSED - 트래킹 시작!"))
                                self.root.after(0, lambda: self.vr_teleop_status.config(text="[VR 활성]", foreground="green"))

                            elif session_state == xr.SessionState.STOPPING:
                                # 세션 종료
                                xr.end_session(self.xr_session)
                                session_running = False

                            elif session_state in (xr.SessionState.LOSS_PENDING, xr.SessionState.EXITING):
                                session_running = False
                                self.root.after(0, lambda: self.log_real("[XR] 세션 종료됨"))
                                return

                except xr.EventUnavailable:
                    pass

                # 세션이 실행 중이 아니면 대기
                if not session_running:
                    time.sleep(0.1)
                    continue

                # 프레임 제출 시도 (FOCUSED가 아니어도 시도해야 WiVRn이 스트리밍 시작)
                frame_began = False
                try:
                    frame_state = xr.wait_frame(self.xr_session)
                    if frame_count < 10:
                        vr_log(f"wait_frame 성공")
                    xr.begin_frame(self.xr_session)
                    frame_began = True
                    frame_count += 1
                    if frame_count <= 10 or frame_count % 100 == 0:
                        vr_log(f"프레임 {frame_count} 시작, 상태={session_state}, should_render={frame_state.should_render}")
                except Exception as frame_err:
                    # 오류 발생 시 짧게 대기 후 재시도
                    err_msg = str(frame_err).lower()
                    if frame_count < 20:
                        vr_log(f"wait/begin 오류: {frame_err}")
                    if "focused" in err_msg or "not running" in err_msg:
                        time.sleep(0.02)
                        continue
                    # 다른 오류도 계속 진행 (WiVRn에서는 정상)
                    time.sleep(0.01)
                    continue

                # 액션 동기화 (FOCUSED 상태가 아니면 실패 가능)
                try:
                    sync_info = xr.ActionsSyncInfo(
                        active_action_sets=[
                            xr.ActiveActionSet(action_set=self.xr_action_set)
                        ]
                    )
                    xr.sync_actions(self.xr_session, sync_info)
                    if frame_count <= 5:
                        vr_log(f"sync_actions 성공")
                except Exception as sync_err:
                    if frame_count <= 10:
                        vr_log(f"sync_actions: {sync_err}")
                    # FOCUSED 상태가 아니면 정상적으로 실패함 - 계속 진행

                pose_data = {}
                button_data = {}

                # 헤드셋(HMD) 위치 가져오기
                hmd_pos_vr = None
                try:
                    hmd_location = xr.locate_space(
                        self.xr_view_space,
                        self.xr_space,
                        frame_state.predicted_display_time
                    )
                    if hmd_location.location_flags & xr.SpaceLocationFlags.POSITION_VALID_BIT:
                        hp = hmd_location.pose.position
                        hmd_pos_vr = (hp.x, hp.y, hp.z)
                except:
                    pass

                for arm in ['right', 'left']:
                    hand_path = self.xr_hand_paths[arm]
                    hand_space = self.xr_hand_spaces[arm]

                    # 포즈 가져오기 (위치 + 회전)
                    try:
                        space_location = xr.locate_space(
                            hand_space,
                            self.xr_space,
                            frame_state.predicted_display_time
                        )

                        if space_location.location_flags & xr.SpaceLocationFlags.POSITION_VALID_BIT:
                            if not connected:
                                connected = True
                                self.root.after(0, lambda: self.vr_teleop_status.config(text="[VR 연결됨]", foreground="green"))
                                self.root.after(0, lambda: self.log_real("[VR] 컨트롤러 트래킹 시작!"))

                            pos = space_location.pose.position
                            orient = space_location.pose.orientation
                            pose_data[arm] = {
                                'position': (pos.x, pos.y, pos.z),
                                'orientation': (orient.x, orient.y, orient.z, orient.w)
                            }
                    except:
                        pass

                    # 그립 상태 (아날로그 값, 0.5 이상이면 pressed로 간주)
                    try:
                        grip_info = xr.ActionStateGetInfo(
                            action=self.xr_grip_action,
                            subaction_path=hand_path
                        )
                        grip_state = xr.get_action_state_float(self.xr_session, grip_info)
                        grip_value = grip_state.current_state if grip_state.is_active else 0.0
                        grip_pressed = grip_value > 0.5
                    except:
                        grip_pressed = False
                        grip_value = 0.0

                    # 트리거 상태
                    try:
                        trigger_info = xr.ActionStateGetInfo(
                            action=self.xr_trigger_action,
                            subaction_path=hand_path
                        )
                        trigger_state = xr.get_action_state_float(self.xr_session, trigger_info)
                        trigger_value = trigger_state.current_state if trigger_state.is_active else 0.0
                    except:
                        trigger_value = 0.0

                    button_data[arm] = {
                        'grip': grip_pressed,
                        'trigger': trigger_value
                    }

                # 스왑체인 처리 및 프레임 종료
                layers = []
                try:
                    if frame_state.should_render and hasattr(self, 'xr_swapchains'):
                        from OpenGL import GL

                        # 뷰 위치 가져오기
                        view_state, views = xr.locate_views(
                            self.xr_session,
                            xr.ViewLocateInfo(
                                view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO,
                                display_time=frame_state.predicted_display_time,
                                space=self.xr_space
                            )
                        )

                        # 각 눈에 대해 스왑체인 처리
                        for i, swapchain in enumerate(self.xr_swapchains):
                            # 스왑체인 이미지 획득
                            image_index = xr.acquire_swapchain_image(
                                swapchain,
                                xr.SwapchainImageAcquireInfo()
                            )

                            # 이미지 준비 대기
                            xr.wait_swapchain_image(
                                swapchain,
                                xr.SwapchainImageWaitInfo(timeout=xr.INFINITE_DURATION)
                            )

                            # 간단한 검은색 배경 렌더링 (텍스처에 직접)
                            # WiVRn은 이것만으로도 스트리밍 시작
                            image = self.xr_swapchain_images[i][image_index]
                            # 렌더링은 생략 - 기본 검은색 유지

                            # 스왑체인 이미지 해제
                            xr.release_swapchain_image(
                                swapchain,
                                xr.SwapchainImageReleaseInfo()
                            )

                            # 프로젝션 뷰 업데이트
                            if i < len(views):
                                self.xr_projection_views[i].pose = views[i].pose
                                self.xr_projection_views[i].fov = views[i].fov

                        # 프로젝션 레이어 생성 및 포인터 배열 만들기
                        projection_layer = xr.CompositionLayerProjection(
                            space=self.xr_space,
                            views=self.xr_projection_views
                        )
                        # ctypes 포인터 배열로 변환
                        layer_ptr = ctypes.cast(
                            ctypes.pointer(projection_layer),
                            ctypes.POINTER(xr.CompositionLayerBaseHeader)
                        )
                        layers = (ctypes.POINTER(xr.CompositionLayerBaseHeader) * 1)(layer_ptr)

                        if frame_count <= 5:
                            vr_log(f"렌더링 완료, 레이어 준비됨")

                except Exception as render_err:
                    if frame_count <= 10:
                        vr_log(f"렌더링 오류: {render_err}")
                    layers = None

                # 프레임 종료
                try:
                    if layers is not None:
                        # 레이어가 있으면 제출
                        frame_end_info = xr.FrameEndInfo(
                            display_time=frame_state.predicted_display_time,
                            environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                        )
                        frame_end_info.layer_count = 1
                        frame_end_info.layers = layers
                        xr.end_frame(self.xr_session, frame_end_info)
                    else:
                        # 레이어 없이 빈 프레임 제출
                        xr.end_frame(self.xr_session, xr.FrameEndInfo(
                            display_time=frame_state.predicted_display_time,
                            environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                            layers=[]
                        ))
                    if frame_count <= 10:
                        vr_log(f"프레임 {frame_count} 종료 성공")
                except Exception as end_err:
                    if frame_count <= 10:
                        vr_log(f"end_frame: {end_err}")
                    # discarded는 정상 - 계속 진행

                # 포즈 처리
                if pose_data:
                    self._process_vr_pose(pose_data, button_data, hmd_pos_vr)

                # 버튼 처리 (그리퍼)
                if button_data:
                    self._process_vr_buttons(button_data)

                time.sleep(0.011)  # ~90Hz

            except Exception as e:
                self.root.after(0, lambda err=str(e): self.log_real(f"[VR] 오류: {err}"))
                time.sleep(0.1)

        # 종료 시 OpenXR 정리
        vr_log(f"스레드 종료, 총 프레임: {frame_count}")
        vr_debug_file.close()
        self._cleanup_openxr()
        self.root.after(0, lambda: self.log_real("[VR] OpenXR 수신 스레드 종료"))

    def _cleanup_openxr(self):
        """OpenXR 리소스 정리"""
        try:
            import xr

            # 스왑체인 정리
            if hasattr(self, 'xr_swapchains'):
                for swapchain in self.xr_swapchains:
                    try:
                        xr.destroy_swapchain(swapchain)
                    except:
                        pass
                self.xr_swapchains = []
                self.xr_swapchain_images = []

            # 액션 스페이스 정리
            if hasattr(self, 'xr_hand_spaces'):
                for space in self.xr_hand_spaces.values():
                    try:
                        xr.destroy_space(space)
                    except:
                        pass
                self.xr_hand_spaces = {}

            # 레퍼런스 스페이스 정리
            if hasattr(self, 'xr_view_space') and self.xr_view_space:
                try:
                    xr.destroy_space(self.xr_view_space)
                except:
                    pass
                self.xr_view_space = None
            if hasattr(self, 'xr_space') and self.xr_space:
                try:
                    xr.destroy_space(self.xr_space)
                except:
                    pass
                self.xr_space = None

            # 액션 정리
            for action_name in ['xr_pose_action', 'xr_grip_action', 'xr_trigger_action']:
                if hasattr(self, action_name) and getattr(self, action_name):
                    try:
                        xr.destroy_action(getattr(self, action_name))
                    except:
                        pass
                    setattr(self, action_name, None)

            # 액션 세트 정리
            if hasattr(self, 'xr_action_set') and self.xr_action_set:
                try:
                    xr.destroy_action_set(self.xr_action_set)
                except:
                    pass
                self.xr_action_set = None

            # 세션 정리
            if hasattr(self, 'xr_session') and self.xr_session:
                try:
                    xr.end_session(self.xr_session)
                except:
                    pass
                try:
                    xr.destroy_session(self.xr_session)
                except:
                    pass
                self.xr_session = None

            # 인스턴스 정리
            if hasattr(self, 'xr_instance') and self.xr_instance:
                try:
                    xr.destroy_instance(self.xr_instance)
                except:
                    pass
                self.xr_instance = None

        except:
            pass

        # GLFW 정리
        if hasattr(self, '_glfw_window') and self._glfw_window:
            try:
                import glfw
                glfw.destroy_window(self._glfw_window)
                glfw.terminate()
            except:
                pass
            self._glfw_window = None

        # 그래픽 바인딩 참조 해제
        if hasattr(self, '_graphics_binding'):
            self._graphics_binding = None

        self.xr_session = None
        self.xr_instance = None

    def _quaternion_to_rpy(self, qx, qy, qz, qw):
        """쿼터니언 → Roll, Pitch, Yaw 변환"""
        import math
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _process_vr_pose(self, pose_data, button_data, hmd_pos_vr=None):
        """OpenXR 포즈 데이터 처리 (위치 + 회전, 헤드셋 상대 좌표)"""
        import math

        # 양쪽 트리거 동시 누름 확인 (캘리브레이션용)
        left_trigger = button_data.get('left', {}).get('trigger', 0.0)
        right_trigger = button_data.get('right', {}).get('trigger', 0.0)
        both_triggers = left_trigger > 0.8 and right_trigger > 0.8

        # 양쪽 포즈 데이터 임시 저장
        arm_data = {}

        for arm in ['right', 'left']:
            if arm not in pose_data:
                continue

            pos = pose_data[arm].get('position', None)
            orient = pose_data[arm].get('orientation', None)
            if pos is None:
                continue

            # 헤드셋 상대 좌표 계산 (컨트롤러 - 헤드셋)
            # 이렇게 하면 회전만 했을 때 위치 변화가 없어짐
            vr_x, vr_y, vr_z = pos
            if hmd_pos_vr is not None:
                vr_x -= hmd_pos_vr[0]
                vr_y -= hmd_pos_vr[1]
                vr_z -= hmd_pos_vr[2]

            # OpenXR 좌표계 → 로봇 좌표계 변환
            # OpenXR: Y=위, Z=뒤, X=오른쪽
            # 로봇: Z=위, X=앞, Y=왼쪽
            x = -vr_z  # OpenXR -Z → 로봇 X (앞)
            y = -vr_x  # OpenXR -X → 로봇 Y (왼쪽)
            z = vr_y   # OpenXR Y → 로봇 Z (위)

            # 회전 변환 (쿼터니언 → RPY)
            if orient:
                qx, qy, qz, qw = orient
                vr_roll, vr_pitch, vr_yaw = self._quaternion_to_rpy(qx, qy, qz, qw)

                # 디버그: 원본 쿼터니언과 변환된 오일러 출력 (100프레임마다, 파일로 저장)
                if not hasattr(self, '_vr_quat_debug_cnt'):
                    self._vr_quat_debug_cnt = 0
                    self._vr_rot_debug_file = open("vr_rotation_debug.txt", "w")
                self._vr_quat_debug_cnt += 1
                if self._vr_quat_debug_cnt % 100 == 0 and arm == 'right':
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self._vr_rot_debug_file.write(f"[{ts}] [VR Quat {arm}] qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}\n")
                    self._vr_rot_debug_file.write(f"[{ts}] [VR Euler {arm}] roll={math.degrees(vr_roll):.1f}, pitch={math.degrees(vr_pitch):.1f}, yaw={math.degrees(vr_yaw):.1f} deg\n")
                    self._vr_rot_debug_file.flush()

                # 좌표계 변환: OpenXR → 로봇
                # OpenXR: X=오른쪽, Y=위, Z=뒤 (-Z가 앞)
                # 로봇: X=앞, Y=왼쪽, Z=위
                # OpenXR 회전:
                #   vr_roll (X축) = 컨트롤러 위아래 기울임 (노딩)
                #   vr_pitch (Y축) = 컨트롤러 좌우 회전 (고개 흔들기)
                #   vr_yaw (Z축) = 컨트롤러 롤링 (손목 돌리기)
                if arm == 'right':
                    # 오른팔: 표준 변환
                    robot_roll = vr_yaw    # VR yaw(Z축, 롤링) → 로봇 roll(전완 회전, servo 15)
                    robot_pitch = vr_roll   # VR roll(X축, 위아래) → 로봇 pitch(손목 상하, servo 16)
                    robot_yaw = vr_pitch    # VR pitch(Y축, 좌우) → 로봇 yaw(손목 좌우, servo 17)
                else:
                    # 왼팔: 미러링 (Y축 기준 반전)
                    robot_roll = -vr_yaw   # 반전
                    robot_pitch = vr_roll   # 유지 (위아래는 같은 방향)
                    robot_yaw = -vr_pitch   # 반전 (좌우는 미러)
            else:
                robot_roll, robot_pitch, robot_yaw = 0.0, 0.0, 0.0

            # 디버그: 변환된 로봇 회전값 출력 (100프레임마다, 파일로 저장)
            if self._vr_quat_debug_cnt % 100 == 0 and arm == 'right':
                from datetime import datetime
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self._vr_rot_debug_file.write(f"[{ts}] [Robot RPY {arm}] roll={math.degrees(robot_roll):.1f}, pitch={math.degrees(robot_pitch):.1f}, yaw={math.degrees(robot_yaw):.1f} deg\n")
                self._vr_rot_debug_file.flush()

            arm_data[arm] = {
                'x': x, 'y': y, 'z': z,
                'roll': robot_roll, 'pitch': robot_pitch, 'yaw': robot_yaw
            }

        # 양쪽 트리거 동시 누름 → 양쪽 동시 캘리브레이션
        if both_triggers and not self.vr_calibrated['right'] and not self.vr_calibrated['left']:
            if 'right' in arm_data and 'left' in arm_data:
                for arm in ['right', 'left']:
                    solver = self.ik_solver.get(arm)
                    if solver:
                        # 활성 baseline 사용
                        active_baseline = self.vr_baseline if self.active_baseline_type == 'vr' else self.robot_baseline
                        servo_values = {}
                        for sid in solver.servo_ids:
                            servo_values[sid] = active_baseline.get(sid, 2048)
                        robot_pos = solver.get_end_effector_position(servo_values)

                        d = arm_data[arm]
                        self.vr_base_pose[arm] = [d['x'], d['y'], d['z']]
                        self.vr_offset[arm] = [robot_pos[0], robot_pos[1], robot_pos[2]]

                        # 회전 캘리브레이션
                        self.vr_base_orientation[arm] = [d['roll'], d['pitch'], d['yaw']]
                        wrist_ids = [15, 16, 17] if arm == 'right' else [25, 26, 27]
                        current_wrist = []
                        for wid in wrist_ids:
                            current_wrist.append(active_baseline.get(wid, 2048))
                        self.vr_rotation_offset[arm] = current_wrist

                        self.vr_calibrated[arm] = True

                self.root.after(0, lambda: self.log_real("[VR] 양쪽 캘리브레이션 완료! (위치+회전)"))
                self.root.after(0, lambda: self.vr_teleop_status.config(text="[VR 활성]", foreground="green"))

        # VR 위치/회전 → 로봇 (캘리브레이션 완료 후)
        for arm in ['right', 'left']:
            if arm not in arm_data:
                continue
            if not self.vr_calibrated[arm] or not self.vr_base_pose[arm]:
                continue
            # 팔 선택 필터링 (all이 아니면 선택된 팔만 처리)
            arm_select = self.vr_arm_select.get()
            if arm_select != "all" and arm_select != arm:
                continue

            d = arm_data[arm]

            # VR baseline 오프셋 계산 (VR baseline 위치 - robot baseline 위치)
            # IK 슬라이더는 robot baseline 기준이므로, VR baseline일 때 이 오프셋 필요
            vr_baseline_offset = [0.0, 0.0, 0.0]
            if self.vr_offset[arm] and self.ik_baseline_xyz[arm]:
                vr_baseline_offset[0] = self.vr_offset[arm][0] - self.ik_baseline_xyz[arm]['x']
                vr_baseline_offset[1] = self.vr_offset[arm][1] - self.ik_baseline_xyz[arm]['y']
                vr_baseline_offset[2] = self.vr_offset[arm][2] - self.ik_baseline_xyz[arm]['z']

            # 위치 델타 계산 (d['x/y/z']는 이미 로봇 좌표계로 변환됨)
            # 좌표계 변환은 이미 위에서 완료됨 (lines 4742-4744)
            robot_dx = d['x'] - self.vr_base_pose[arm][0]
            robot_dy = d['y'] - self.vr_base_pose[arm][1]
            robot_dz = d['z'] - self.vr_base_pose[arm][2]

            # 스케일 적용 + VR baseline 오프셋
            dx = robot_dx * self.vr_scale + vr_baseline_offset[0]
            dy = robot_dy * self.vr_scale + vr_baseline_offset[1]
            dz = robot_dz * self.vr_scale + vr_baseline_offset[2]

            # 회전 델타 (라디안)
            if self.vr_base_orientation[arm]:
                d_roll = d['roll'] - self.vr_base_orientation[arm][0]
                d_pitch = d['pitch'] - self.vr_base_orientation[arm][1]
                d_yaw = d['yaw'] - self.vr_base_orientation[arm][2]

                # 디버그: 100프레임마다 VR 회전 델타 출력 (파일로 저장)
                if hasattr(self, '_vr_rot_debug_file') and self._vr_quat_debug_cnt % 100 == 0 and arm == 'right':
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self._vr_rot_debug_file.write(f"[{ts}] [VR Delta] cur=({math.degrees(d['roll']):.1f}, {math.degrees(d['pitch']):.1f}, {math.degrees(d['yaw']):.1f}) "
                          f"base=({math.degrees(self.vr_base_orientation[arm][0]):.1f}, {math.degrees(self.vr_base_orientation[arm][1]):.1f}, {math.degrees(self.vr_base_orientation[arm][2]):.1f}) "
                          f"delta=({math.degrees(d_roll):.1f}, {math.degrees(d_pitch):.1f}, {math.degrees(d_yaw):.1f}) deg\n")
                    self._vr_rot_debug_file.flush()
            else:
                d_roll, d_pitch, d_yaw = 0.0, 0.0, 0.0

            # dx/dy/dz는 robot baseline 기준 델타값
            self.root.after(0, lambda a=arm, rx=dx, ry=dy, rz=dz,
                           dr=d_roll, dp=d_pitch, dy_=d_yaw:
                           self._apply_vr_to_ik_with_rotation(a, rx, ry, rz, dr, dp, dy_))

    def _process_vr_buttons(self, button_data):
        """OpenXR 버튼 데이터 처리 (그리퍼 등)"""
        for arm in ['right', 'left']:
            if arm not in button_data:
                continue

            if not self.vr_calibrated[arm]:
                continue

            # 팔 선택 필터링 (all이 아니면 선택된 팔만 처리)
            arm_select = self.vr_arm_select.get()
            if arm_select != "all" and arm_select != arm:
                continue

            trigger = button_data[arm].get('trigger', 0.0)

            # 트리거 → 그리퍼 제어 (캘리브레이션 완료 후에만)
            gripper_id = 41 if arm == 'right' else 31  # RX-1 그리퍼 ID
            # baseline = 0 (열림), 트리거 누르면 닫힘 (값 증가)
            # 트리거 0% = 그리퍼 0 (열림), 트리거 100% = 그리퍼 4095 (닫힘)
            gripper_baseline = self.robot_baseline.get(gripper_id, 10)
            gripper_pos = int(gripper_baseline + trigger * 4025)
            gripper_pos = max(10, min(4035, gripper_pos))

            self.root.after(0, lambda gid=gripper_id, gpos=gripper_pos:
                           self._apply_vr_gripper(gid, gpos))

    def _apply_vr_to_ik_with_rotation(self, arm, x, y, z, d_roll, d_pitch, d_yaw):
        """VR 위치+회전을 6-DOF IK로 적용"""
        if not self.vr_enabled:
            return

        solver = self.ik_solver.get(arm)
        if not solver:
            return

        # 위치 델타 저장
        self._ik_updating = True
        try:
            if 'x' in self.ik_sliders[arm]:
                self.ik_sliders[arm]['x'].set(x)
            if 'y' in self.ik_sliders[arm]:
                self.ik_sliders[arm]['y'].set(y)
            if 'z' in self.ik_sliders[arm]:
                self.ik_sliders[arm]['z'].set(z)

            self.ik_values[arm]['x'] = x
            self.ik_values[arm]['y'] = y
            self.ik_values[arm]['z'] = z

            # 방향 델타 저장 (6-DOF IK용)
            self.ik_orientation_delta[arm]['roll'] = d_roll
            self.ik_orientation_delta[arm]['pitch'] = d_pitch
            self.ik_orientation_delta[arm]['yaw'] = d_yaw

            # 값 라벨 업데이트
            if 'x' in self.ik_value_labels[arm]:
                self.ik_value_labels[arm]['x'].config(text=f"{x:.3f}")
            if 'y' in self.ik_value_labels[arm]:
                self.ik_value_labels[arm]['y'].config(text=f"{y:.3f}")
            if 'z' in self.ik_value_labels[arm]:
                self.ik_value_labels[arm]['z'].config(text=f"{z:.3f}")
        finally:
            self._ik_updating = False

        # 6-DOF IK 계산 트리거
        self._schedule_ik_update(arm)

    def _schedule_ik_update(self, arm):
        """VR에서 IK 계산 스케줄링 (팔별 독립 처리)"""
        # 이전 펜딩 콜백 취소 (큐 누적 방지)
        if self.vr_ik_pending[arm]:
            try:
                self.root.after_cancel(self.vr_ik_pending[arm])
            except:
                pass
        # 최신 콜백만 스케줄링
        self.vr_ik_pending[arm] = self.root.after(10, lambda a=arm: self._do_vr_ik_update(a))

    def _do_vr_ik_update(self, arm):
        """VR IK update: 6-DOF IK (위치 + 방향)"""
        solver = self.ik_solver.get(arm)
        if not solver:
            return

        import numpy as np

        # 위치 델타
        delta_xyz = [
            self.ik_values[arm]['x'],
            self.ik_values[arm]['y'],
            self.ik_values[arm]['z']
        ]

        # 방향 델타
        delta_rpy = [
            self.ik_orientation_delta[arm]['roll'],
            self.ik_orientation_delta[arm]['pitch'],
            self.ik_orientation_delta[arm]['yaw']
        ]

        # baseline 위치 및 방향
        base_xyz = self.ik_baseline_xyz[arm]
        base_rpy = self.ik_baseline_rpy[arm]

        # 현재 서보값을 초기값으로 사용 (연속성 유지, 튀는 현상 방지)
        # 캐시된 값이 있으면 그걸 사용, 없으면 baseline
        active_baseline = self.vr_baseline if self.active_baseline_type == 'vr' else self.robot_baseline
        if self.vr_last_valid_servos[arm]:
            servo_values = self.vr_last_valid_servos[arm].copy()
        else:
            servo_values = {}
            for sid in solver.servo_ids:
                servo_values[sid] = active_baseline.get(sid, 2048)

        # 목표 위치: baseline + delta
        target_xyz = [
            base_xyz['x'] + delta_xyz[0],
            base_xyz['y'] + delta_xyz[1],
            base_xyz['z'] + delta_xyz[2]
        ]

        # 목표 방향: baseline + delta
        target_rpy = [
            base_rpy['roll'] + delta_rpy[0],
            base_rpy['pitch'] + delta_rpy[1],
            base_rpy['yaw'] + delta_rpy[2]
        ]

        # 실제 하드웨어로 전송할 서보 위치 모음
        real_servo_targets = {}

        # 위치 전용 IK (손목 회전은 직접 매핑하므로 rot_weight=0)
        # joint_weights: 높을수록 해당 관절이 안 움직임
        #   joints 0-3 (어깨/팔꿈치): 높은 저항 → 치킨윙 방지
        #   joints 4-6 (손목): 매우 높은 저항 → IK가 손목을 안 건드림 (직접 매핑)
        import numpy as np
        vr_joint_weights = np.array([5.0, 3.0, 5.0, 2.0, 50.0, 50.0, 50.0])
        new_servos, success = solver.solve_for_pose(
            target_xyz, target_rpy, servo_values,
            max_iterations=50,
            pos_weight=1.0,
            rot_weight=0.0,
            joint_weights=vr_joint_weights
        )

        # 손목 서보 직접 매핑: VR 회전 delta → 서보값 (IK 결과 덮어쓰기)
        wrist_ids = [15, 16, 17] if arm == 'right' else [25, 26, 27]
        rotation_offset = self.vr_rotation_offset[arm]
        if rotation_offset and any(v != 0 for v in rotation_offset):
            import math
            ticks_360 = 4095.0 / (2.0 * math.pi)
            ticks_120 = 4095.0 / (120.0 * math.pi / 180.0)
            inversions = solver.viz_inversions[4:7]

            # 오른손 회전 delta 반전 (오른손만 3축 모두 반대)
            dr = delta_rpy
            if arm == 'right':
                dr = [-delta_rpy[0], -delta_rpy[1], -delta_rpy[2]]

            yaw_scale = 1.0 / 3.0  # 좌우 회전 감도 1/3
            pitch_scale = 0.5      # 상하 회전 감도 1/2
            wrist_direct = {
                wrist_ids[0]: max(0, min(4095, int(rotation_offset[0] - inversions[0] * dr[0] * ticks_360))),
                wrist_ids[1]: max(0, min(4095, int(rotation_offset[1] + inversions[1] * dr[1] * pitch_scale * ticks_120))),
                wrist_ids[2]: max(0, min(4095, int(rotation_offset[2] + inversions[2] * dr[2] * yaw_scale * ticks_120))),
            }

            # IK 실패해도 손목은 항상 적용
            if not new_servos:
                new_servos = {}
                if self.vr_last_valid_servos[arm]:
                    for sid in solver.servo_ids[:4]:
                        if sid in self.vr_last_valid_servos[arm]:
                            new_servos[sid] = self.vr_last_valid_servos[arm][sid]
                success = True

            # 손목 서보 덮어쓰기
            new_servos.update(wrist_direct)

        if success and new_servos:
            SERVO_MIN = 50
            SERVO_MAX = 4045
            MAX_DELTA = 150  # 한 프레임당 최대 변화량 (튀는 현상 방지)

            for sid in new_servos:
                # 급격한 변화 제한 (이전 값과 비교)
                if self.vr_last_valid_servos[arm] and sid in self.vr_last_valid_servos[arm]:
                    prev_val = self.vr_last_valid_servos[arm][sid]
                    delta = new_servos[sid] - prev_val
                    if abs(delta) > MAX_DELTA:
                        new_servos[sid] = prev_val + (MAX_DELTA if delta > 0 else -MAX_DELTA)

                # 리밋 클램핑
                if new_servos[sid] < SERVO_MIN:
                    new_servos[sid] = SERVO_MIN
                elif new_servos[sid] > SERVO_MAX:
                    new_servos[sid] = SERVO_MAX

            # 캐시 업데이트
            self.vr_last_valid_servos[arm] = new_servos.copy()
            self.vr_last_valid_xyz[arm] = target_xyz.copy()

        if success and new_servos:
            for sid in solver.servo_ids:
                if sid in new_servos:
                    val = int(max(50, min(4045, new_servos[sid])))
                    if sid in self.sim_active_joints:
                        self.sim_active_joints[sid]['current'] = val
                    if sid in self.real_active_joints:
                        self.real_active_joints[sid]['current'] = val
                        real_servo_targets[sid] = val

        # 디버그 카운터
        if hasattr(self, '_vr_wrist_debug_cnt'):
            self._vr_wrist_debug_cnt += 1
        else:
            self._vr_wrist_debug_cnt = 0

        # 실제 하드웨어로 명령 전송 (Sync Write)
        if real_servo_targets and self.real_connected and self.real_time_enabled.get():
            try:
                self.sync_write_positions(real_servo_targets)
            except Exception as e:
                pass

        self.schedule_sim_update()

    def _apply_vr_gripper(self, gripper_id, position):
        """VR 트리거 → 그리퍼 제어"""
        position = max(10, min(4035, position))
        if gripper_id in self.real_sliders:
            self.real_sliders[gripper_id].set(position)
        if gripper_id in self.sim_active_joints:
            self.sim_active_joints[gripper_id]['current'] = position
        if gripper_id in self.real_active_joints:
            self.real_active_joints[gripper_id]['current'] = position
            # 실제 하드웨어로 그리퍼 명령 전송
            if self.real_connected and self.real_time_enabled.get():
                try:
                    self.send_real_servo_command(gripper_id, position)
                except Exception:
                    pass
        self.schedule_sim_update()

    def stop_vr_teleop(self):
        """VR 텔레옵 중지 (OpenXR 종료)"""
        # 연결 대기 중이면 취소 플래그 설정
        self._vr_cancel_wait = True

        self.vr_enabled = False
        self.vr_calibrated = {'right': False, 'left': False}
        self.vr_base_pose = {'right': None, 'left': None}

        # VR 스레드 종료 대기
        if self.vr_thread and self.vr_thread.is_alive():
            self.vr_thread.join(timeout=1.0)
        self.vr_thread = None

        # OpenXR 종료
        self._cleanup_openxr()
        self.log_real("[OK] OpenXR 종료됨")

        self.vr_teleop_btn.config(text="VR 텔레옵")
        self.vr_teleop_status.config(text="[VR 꺼짐]", foreground="gray")
        self.log_real("[OK] VR 텔레옵 중지됨")

    def _get_local_ip(self):
        """로컬 IP 주소 반환"""
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"

    def _start_isaac_sim(self):
        """Isaac ROS 프로세스 시작 (RViz + robot_state_publisher)"""
        try:
            # 이미 실행 중이면 다시 시작하지 않음
            if self.sim_process and self.sim_process.poll() is None:
                self.log_real("[WARN] Isaac ROS가 이미 실행 중입니다. 재시작하지 않습니다.")
                return

            self.log_real("Isaac ROS 시작 중...")

            script_path = os.path.join(_GUI_ROOT, "start_isaac_ros_full.sh")

            if not os.path.exists(script_path):
                self.log_real(f"[ERR] Isaac ROS 스크립트를 찾을 수 없음: {script_path}")
                return

            # 로그 파일 생성
            log_file_path = os.path.join(_GUI_ROOT, "isaac_ros_bridge.log")
            self.sim_log_file = open(log_file_path, 'w', encoding='utf-8')

            self.sim_process = subprocess.Popen(
                ["bash", script_path],
                stdout=self.sim_log_file,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            self.log_real("[OK] Isaac ROS 시작됨 (RViz + robot_state_publisher)")
            self.log_real(f"   로그: {log_file_path}")

            # Sim 연결 상태 설정
            self.sim_connected = True

            # GUI 상태 업데이트
            self.sim_status.config(text="[O] Sim 켜짐", foreground="green")
            self.log_real("   RX-1 로봇 시각화 준비 중...")

            # 로그 파일 모니터링 (별도 스레드)
            threading.Thread(target=self._monitor_sim_log, args=(log_file_path,), daemon=True).start()

        except Exception as e:
            self.log_real(f"[ERR] Isaac Sim 시작 실패: {e}")

    def _monitor_sim_log(self, log_file_path):
        """Isaac Sim 로그 파일 모니터링"""
        import time
        try:
            with open(log_file_path, 'r', encoding='utf-8') as f:
                f.seek(0, 2)  # 파일 끝으로 이동
                while self.sim_process and self.sim_process.poll() is None:
                    line = f.readline()
                    if line:
                        line = line.strip()
                        if line:
                            # 중요한 메시지만 GUI에 표시
                            if any(keyword in line for keyword in ['[Camera]', '[WHEEL]', '[SERVO]', '[Lift]', 'OK]', 'ERROR]', 'DriveAPI', 'configured']):
                                self.log_real(f"[Sim] {line}")
                    else:
                        time.sleep(0.1)
        except Exception as e:
            self.log_real(f"[Sim] Log monitoring error: {e}")

    def disconnect_sim(self):
        """Sim 연결 해제"""
        if self.sim_process:
            try:
                self.sim_process.terminate()
                self.sim_process.wait(timeout=5)
            except:
                self.sim_process.kill()
            self.sim_process = None

        if hasattr(self, 'sim_log_file'):
            try:
                self.sim_log_file.close()
            except:
                pass

        self.sim_connected = False
        self.log_real("Sim 연결 해제됨")

    def schedule_sim_update(self):
        """Sim 업데이트를 스로틀링하여 50ms마다 한 번만 실행"""
        current_time = time.time()

        # 이미 대기 중인 업데이트가 있으면 건너뛰기
        if self._sim_update_pending:
            return

        # 마지막 업데이트 후 50ms가 지났으면 바로 실행
        if current_time - self._last_sim_update_time >= 0.05:
            self._last_sim_update_time = current_time
            self.update_sim()
        else:
            # 아니면 50ms 후에 실행 예약
            self._sim_update_pending = True
            delay_ms = int((0.05 - (current_time - self._last_sim_update_time)) * 1000) + 1
            self.root.after(delay_ms, self._execute_pending_sim_update)

    def _execute_pending_sim_update(self):
        """예약된 Sim 업데이트 실행"""
        self._sim_update_pending = False
        self._last_sim_update_time = time.time()
        self.update_sim()

    def update_sim(self):
        """Sim 업데이트 (명령 파일로 전송)"""
        if not (self.sim_connected ):
            # 서보 26, 27 디버그: sim_connected 체크
            if hasattr(self, '_sim_debug_cnt'):
                self._sim_debug_cnt += 1
            else:
                self._sim_debug_cnt = 0
            if self._sim_debug_cnt % 50 == 0:
                from datetime import datetime
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                with open("servo_26_27_debug.txt", "a") as f:
                    f.write(f"[{ts}] [UPDATE_SIM SKIP] sim_connected={self.sim_connected}\n")
            return

        try:
            INVERTED_SERVOS = [21, 22]
            NO_GEARBOX_SERVOS = [15, 25]  # 360도 회전 가능 (기어비 없음)
            REASSEMBLED_SERVOS = [24]  # 하드웨어 재조립으로 방향 반전 (L Elbow만)
            GRIPPER_SERVOS = [28, 31, 32, 41]  # 그리퍼 서보 (Prismatic joint, 0~4095 -> 0~0.04m (baseline 0))
            RVIZ_AUTO_INVERTED = [14]  # RViz에서 자동 반전 (R Elbow, URDF 이전 시 발생한 문제)

            # ROS 전용 베이스라인 (URDF 0도 자세 기준)
            # 원래 기본자세 값 사용하여 새 기본자세가 RVIZ에서 올바르게 표시됨
            # ROS 전용 베이스라인 (URDF 0도 자세 기준)
            # 원래 기본자세 값 사용하여 새 기본자세가 RVIZ에서 올바르게 표시됨
            ROS_BASELINE_OVERRIDE = {
                11: 2850, 12: 186, 13: 1984, 14: 4095, 15: 2991, 16: 2008, 17: 2680,
                21: 1131, 22: 4003, 23: 1856, 24: 150, 25: 2061, 26: 2139, 27: 1839
            }
            # ROS 전용 심반전 (슬라이더 심반전과 무관, ROS에서만 적용)
            ROS_SIM_INVERTED = [14]

            servo_to_joint = {
                11: "right_shoul_base2shoul_joint_11_",
                12: "right_shoul2shoul_rot_joint_12_",
                13: "right_arm2armrot_joint_13_",
                14: "right_armrot2elbow_joint_14_",
                15: "right_forearm2forearmrot_joint_15_",
                16: "right_forearmrot2forearm_pitch_joint_16_",
                17: "right_forearm_pitch2forearm_roll_joint_17_",
                41: "right_gripper_joint_41_",
                32: "right_gripper_tip2_joint_32_",
                21: "left_shoul_base2shoul_joint_21_",
                22: "left_shoul2shoul_rot_joint_22_",
                23: "left_arm2armrot_joint_23_",
                24: "left_armrot2elbow_joint_24_",
                25: "left_forearm2forearmrot_joint_25_",
                26: "left_forearmrot2forearm_pitch_joint_26_",
                27: "left_forearm_pitch2forearm_roll_joint_27_",
                31: "left_gripper_joint_31_",
                28: "left_gripper_tip2_joint_28_",
            }

            joints_data = []

            # 디버그: 파일에만 출력 (콘솔 스팸 방지)
            self.debug_log(f"[UPDATE_SIM] sim_active_joints[13]['current'] = {self.sim_active_joints.get(13, {}).get('current', 'NONE')}")
            self.debug_log(f"[UPDATE_SIM] sim_active_joints[23]['current'] = {self.sim_active_joints.get(23, {}).get('current', 'NONE')}")
            self.debug_log(f"[UPDATE_SIM] sim_baseline_positions[13] = {self.sim_baseline_positions.get(13, 'NONE')}")
            self.debug_log(f"[UPDATE_SIM] sim_baseline_positions[23] = {self.sim_baseline_positions.get(23, 'NONE')}")

            for sid, jname in servo_to_joint.items():
                # 그리퍼 Tip2는 Tip1의 위치를 따라감 (32->41, 28->31)
                if sid == 32:
                    pos = self.sim_active_joints.get(41, {}).get('current', 0)
                elif sid == 28:
                    pos = self.sim_active_joints.get(31, {}).get('current', 0)
                else:
                    pos = self.sim_active_joints.get(sid, {}).get('current', 0)
                # ROS 전용 베이스라인 사용 (14, 24번만 - 슬라이더와 무관)
                if sid in ROS_BASELINE_OVERRIDE:
                    baseline_pos = ROS_BASELINE_OVERRIDE[sid]
                else:
                    baseline_pos = self.sim_baseline_positions.get(sid, 2048)

                # 실제 로봇의 하드웨어 min/max 범위 체크
                if sid in self.real_active_joints:
                    joint_min = self.real_active_joints[sid]['min']
                    joint_max = self.real_active_joints[sid]['max']

                    if pos < joint_min or pos > joint_max:
                        # 범위를 벗어나면 제한값으로 클램프 (베이스라인 대신)
                        clamped_pos = max(joint_min, min(joint_max, pos))
                        self.debug_log(f"[SIM_LIMIT] 서보 {sid}: 범위 초과 {pos} -> {clamped_pos}, 허용범위 [{joint_min}~{joint_max}]")
                        pos = clamped_pos

                relative_pos = pos - baseline_pos

                # 디버그 로그 (13, 23번 - 파일과 GUI 모두)
                if sid in [13, 23]:
                    debug_msg = f"[DEBUG] 서보 {sid}: pos={pos}, baseline={baseline_pos}, rel_pos={relative_pos}"
                    self.log_real(debug_msg)
                # 서보 26, 27 디버그 (txt 파일로 저장)
                if sid in [26, 27]:
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open("servo_26_27_debug.txt", "a") as f:
                        f.write(f"[{ts}] [ROS] 서보 {sid}: pos={pos}, baseline={baseline_pos}, rel_pos={relative_pos}\n")
                    self.debug_log(debug_msg)


                # 그리퍼 서보는 Prismatic joint (0~4095 -> 0~0.04m)
                if sid in GRIPPER_SERVOS:
                    rel_distance_m = (relative_pos / 4095) * 0.04
                    # 그리퍼 움직임 48% 증가 (기본자세는 유지, 움직이는 양만 증가) - 1.44 * 1.03
                    rel_distance_m = rel_distance_m * 1.4832
                    rel_angle = rel_distance_m * 1000
                elif sid in NO_GEARBOX_SERVOS:
                    rel_angle = (relative_pos / 4095) * 360
                else:
                    rel_angle = (relative_pos / 4095) * 120

                # 서보 14, 24는 하드웨어 재조립으로 방향 반전
                if sid in REASSEMBLED_SERVOS:
                    rel_angle = -rel_angle

                # 하드웨어 반전 (서보 21, 22)
                if sid in INVERTED_SERVOS:
                    rel_angle = -rel_angle

                # RViz 자동 반전 (URDF 이전 과정에서 발생한 문제, Sim 반전 체크 없이 자동 적용)
                if sid in RVIZ_AUTO_INVERTED:
                    rel_angle = -rel_angle

                # 소프트웨어 반전 (Sim만 적용, GUI 반전 버튼)
                if self.sim_inverted.get(sid, False):
                    rel_angle = -rel_angle

                # ROS 전용 심반전 (슬라이더와 무관, ROS에서만 적용)
                if sid in ROS_SIM_INVERTED:
                    rel_angle = -rel_angle

                # 그리퍼 Tip2 (서보 28, 32)는 반대 방향으로 계산
                if sid in [28, 32]:
                    rel_angle = -rel_angle

                # 디버그 로그 (13, 23번 - 파일과 GUI 모두)
                if sid in [13, 23]:
                    debug_msg = f"[DEBUG] 서보 {sid}: rel_angle={rel_angle}°, rad={math.radians(rel_angle)}"
                    self.log_real(debug_msg)
                    self.debug_log(debug_msg)
                # 서보 26, 27 디버그 (txt 파일로 저장)
                if sid in [26, 27]:
                    from datetime import datetime
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with open("servo_26_27_debug.txt", "a") as f:
                        f.write(f"[{ts}] [ROS ANGLE] 서보 {sid}: rel_angle={rel_angle:.2f}°, rad={math.radians(rel_angle):.4f}\n")

                # 손목 서보 디버그 (16, 17 - 오른팔 손목, rel_angle 계산 후)
                if sid in [16, 17]:
                    if not hasattr(self, '_vr_rot_debug_file') or self._vr_rot_debug_file is None:
                        self._vr_rot_debug_file = open("vr_rotation_debug.txt", "a")
                    if not hasattr(self, '_ros_wrist_debug_cnt'):
                        self._ros_wrist_debug_cnt = 0
                    self._ros_wrist_debug_cnt += 1
                    if self._ros_wrist_debug_cnt % 100 == 0:
                        from datetime import datetime
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        self._vr_rot_debug_file.write(f"[{ts}] [ROS servo {sid}] pos={pos}, baseline={baseline_pos}, rel_pos={relative_pos}, rel_angle={rel_angle:.1f}deg\n")
                        self._vr_rot_debug_file.flush()

                # 최종 값 계산 (그리퍼는 미터, 나머지는 라디안)
                if sid in GRIPPER_SERVOS:
                    final_value = rel_angle / 1000.0
                else:
                    final_value = math.radians(rel_angle)

                # 31번 그리퍼 Y축 +1.7cm 오프셋
                if sid == 31:
                    final_value += 0.017

                joints_data.append({
                    "name": jname,
                    "position_rad": final_value,
                    "velocity_rad_s": self.sim_velocity_value.get()
                })

            command_data = {
                "timestamp": time.time(),
                "joints": joints_data,
                "velocity": self.sim_velocity_value.get()
            }

            with open(self.sim_command_file, 'w') as f:
                json.dump(command_data, f)

            # Sim->Real 모드: Sim의 위치를 Real에 전송
            if self.sim_link_to_real and self.real_connected:
                self.updating_from_sim = True  # 무한 루프 방지 플래그 설정
                try:
                    for sid in self.sim_active_joints.keys():
                        if sid in self.real_active_joints:
                            sim_pos = self.sim_active_joints[sid]['current']
                            # Real 내부 값 업데이트
                            self.real_active_joints[sid]['current'] = sim_pos
                            # Real 슬라이더도 업데이트
                            if sid in self.real_sliders:
                                self.real_sliders[sid].set(sim_pos)
                            # Real 모터에 명령 전송
                            if self.real_time_enabled.get():
                                self.send_real_servo_command(sid, sim_pos)
                finally:
                    self.updating_from_sim = False  # 플래그 해제

        except Exception as e:
            self.log_real(f"[ERR] Sim 업데이트 실패: {e}")

    def load_sim_baseline(self):
        """Sim 베이스라인 로드"""
        try:
            baseline_path = os.path.join(_GUI_ROOT, "ros_files", "rx1_baseline.json")

            with open(baseline_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            for servo_id in self.sim_active_joints.keys():
                if str(servo_id) in data['servos']:
                    baseline_pos = data['servos'][str(servo_id)]['position']
                    self.sim_baseline_positions[servo_id] = baseline_pos
                    # current 값도 베이스라인으로 설정 (처음 시작 시 기본 자세)
                    self.sim_active_joints[servo_id]['current'] = baseline_pos

            self.log_real("Sim 베이스라인 로드 완료 (현재 위치도 베이스라인으로 설정)")

        except Exception as e:
            self.log_real(f"Sim 베이스라인 로드 실패: {e}")

    # ===================================================================
    # 유틸리티
    # ===================================================================

    def log_real(self, message):
        """로그 (스레드 안전)"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"

        def _insert():
            try:
                self.real_status_text.insert(tk.END, log_entry)
                self.real_status_text.see(tk.END)
            except:
                pass

        # 스레드에서 호출 시 after() 사용
        try:
            if threading.current_thread() is threading.main_thread():
                _insert()
            else:
                self.root.after(0, _insert)
        except:
            pass

    # ===================================================================
    # 시스템 모니터링
    # ===================================================================

    def _system_monitor_loop(self):
        """시스템 리소스 모니터링 루프 (Jetson 전용)"""
        while self.system_monitor_running:
            try:
                # CPU 사용률
                try:
                    with open('/proc/stat', 'r') as f:
                        line = f.readline()
                        parts = line.split()
                        idle = int(parts[4])
                        total = sum(int(p) for p in parts[1:8])
                        if hasattr(self, '_last_cpu_idle'):
                            idle_delta = idle - self._last_cpu_idle
                            total_delta = total - self._last_cpu_total
                            self.cpu_usage = 100 * (1 - idle_delta / max(1, total_delta))
                        self._last_cpu_idle = idle
                        self._last_cpu_total = total
                except:
                    self.cpu_usage = 0

                # RAM 사용률
                try:
                    with open('/proc/meminfo', 'r') as f:
                        lines = f.readlines()
                        mem_total = int(lines[0].split()[1])
                        mem_available = int(lines[2].split()[1])
                        self.ram_usage = 100 * (1 - mem_available / mem_total)
                        self.ram_mb = (mem_total - mem_available) // 1024
                except:
                    self.ram_usage = 0

                # Jetson GPU 정보 (tegrastats 대신 파일 읽기)
                try:
                    # GPU 로드
                    gpu_load_path = '/sys/devices/gpu.0/load'
                    if os.path.exists(gpu_load_path):
                        with open(gpu_load_path, 'r') as f:
                            self.gpu_usage = int(f.read().strip()) / 10  # 0-1000 -> 0-100
                    else:
                        self.gpu_usage = 0
                except:
                    self.gpu_usage = 0

                # Jetson 온도
                try:
                    # CPU 온도
                    temp_paths = [
                        '/sys/devices/virtual/thermal/thermal_zone0/temp',
                        '/sys/class/thermal/thermal_zone0/temp'
                    ]
                    for path in temp_paths:
                        if os.path.exists(path):
                            with open(path, 'r') as f:
                                self.temp_cpu = int(f.read().strip()) // 1000
                            break

                    # GPU 온도
                    gpu_temp_paths = [
                        '/sys/devices/virtual/thermal/thermal_zone1/temp',
                        '/sys/class/thermal/thermal_zone1/temp'
                    ]
                    for path in gpu_temp_paths:
                        if os.path.exists(path):
                            with open(path, 'r') as f:
                                self.temp_gpu = int(f.read().strip()) // 1000
                            break
                except:
                    pass

                # 서보 온도 (5초마다, 연동 중 아닐 때만)
                if not hasattr(self, '_servo_temp_counter'):
                    self._servo_temp_counter = 0
                self._servo_temp_counter += 1

                if self._servo_temp_counter >= 5:
                    self._servo_temp_counter = 0
                    if self.real_connected and not self.teleop_to_robot_active:
                        self._update_servo_temperatures()

                # UI 업데이트
                self.root.after(0, self._update_system_status_ui)

            except Exception as e:
                pass

            time.sleep(1)  # 1초마다 업데이트

    def _update_system_status_ui(self):
        """시스템 상태 UI 업데이트"""
        try:
            # CPU
            cpu_color = '#00ff00' if self.cpu_usage < 70 else '#ffff00' if self.cpu_usage < 90 else '#ff4444'
            self.cpu_label.config(text=f"{self.cpu_usage:.0f}%", fg=cpu_color)

            # RAM
            ram_color = '#00ff00' if self.ram_usage < 70 else '#ffff00' if self.ram_usage < 90 else '#ff4444'
            self.ram_label.config(text=f"{self.ram_usage:.0f}%", fg=ram_color)

            # GPU
            gpu_color = '#00ff00' if self.gpu_usage < 70 else '#ffff00' if self.gpu_usage < 90 else '#ff4444'
            self.gpu_label.config(text=f"{self.gpu_usage:.0f}%", fg=gpu_color)

            # VRAM (RAM에서 추정)
            self.vram_label.config(text=f"{getattr(self, 'ram_mb', 0)}MB")

            # CPU 온도
            temp_cpu_color = '#00ff00' if self.temp_cpu < 60 else '#ffff00' if self.temp_cpu < 80 else '#ff4444'
            self.temp_cpu_label.config(text=f"{self.temp_cpu}°C", fg=temp_cpu_color)

            # GPU 온도
            temp_gpu_color = '#00ff00' if self.temp_gpu < 60 else '#ffff00' if self.temp_gpu < 80 else '#ff4444'
            self.temp_gpu_label.config(text=f"{self.temp_gpu}°C", fg=temp_gpu_color)

            # 서보 온도 표시
            if hasattr(self, 'servo_temp_right') and self.servo_temp_right:
                temp_r, name_r = self.servo_temp_right
                color_r = '#00ff00' if temp_r < 50 else '#ffff00' if temp_r < 60 else '#ff4444'
                self.servo_temp_right_label.config(text=f"{temp_r}°C {name_r}", fg=color_r)
            elif not self.real_connected:
                self.servo_temp_right_label.config(text="--", fg='gray')

            if hasattr(self, 'servo_temp_left') and self.servo_temp_left:
                temp_l, name_l = self.servo_temp_left
                color_l = '#00ff00' if temp_l < 50 else '#ffff00' if temp_l < 60 else '#ff4444'
                self.servo_temp_left_label.config(text=f"{temp_l}°C {name_l}", fg=color_l)
            elif not self.real_connected:
                self.servo_temp_left_label.config(text="--", fg='gray')

            # 경고 메시지
            warnings = []
            if self.cpu_usage > 90:
                warnings.append("CPU 과부하!")
            if self.ram_usage > 90:
                warnings.append("RAM 부족!")
            if self.temp_cpu > 80 or self.temp_gpu > 80:
                warnings.append("과열 주의!")

            # 서보 과열 경고
            if hasattr(self, 'servo_temp_right') and self.servo_temp_right and self.servo_temp_right[0] >= 60:
                warnings.append(f"R서보 과열!")
            if hasattr(self, 'servo_temp_left') and self.servo_temp_left and self.servo_temp_left[0] >= 60:
                warnings.append(f"L서보 과열!")

            self.warning_label.config(text=" | ".join(warnings))

        except Exception as e:
            pass

    def _update_servo_temperatures(self):
        """서보 온도 읽기 및 저장 (백그라운드)"""
        try:
            right_arm_ids = [11, 12, 13, 14, 15, 16, 17]
            left_arm_ids = [21, 22, 23, 24, 25, 26, 27]

            # 오른팔 최고 온도
            max_temp_r = 0
            max_id_r = None
            for sid in right_arm_ids:
                temp = self.read_servo_temperature(sid)
                if temp is not None and temp > max_temp_r:
                    max_temp_r = temp
                    max_id_r = sid

            if max_id_r:
                name = self.real_active_joints.get(max_id_r, {}).get('name', f'ID{max_id_r}')
                # 짧은 이름으로 변환
                short_name = name.replace('R ', '').replace('L ', '')[:8]
                self.servo_temp_right = (max_temp_r, short_name)
            else:
                self.servo_temp_right = None

            # 왼팔 최고 온도
            max_temp_l = 0
            max_id_l = None
            for sid in left_arm_ids:
                temp = self.read_servo_temperature(sid)
                if temp is not None and temp > max_temp_l:
                    max_temp_l = temp
                    max_id_l = sid

            if max_id_l:
                name = self.real_active_joints.get(max_id_l, {}).get('name', f'ID{max_id_l}')
                short_name = name.replace('R ', '').replace('L ', '')[:8]
                self.servo_temp_left = (max_temp_l, short_name)
            else:
                self.servo_temp_left = None

        except Exception as e:
            pass

    def run(self):
        """GUI 실행"""
        # GUI 종료 시 정리 함수 등록
        self.root.protocol("WM_DELETE_WINDOW", self._on_gui_closing)

        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            print("프로그램 종료")
            self._cleanup_on_exit()

    def _on_gui_closing(self):
        """GUI 창 닫힐 때 호출"""
        self._cleanup_on_exit()
        self.root.destroy()

    def _cleanup_on_exit(self):
        """종료 시 모든 리소스 정리"""
        print("프로그램 종료 - 리소스 정리 중...")

        # 시스템 모니터링 중지
        self.system_monitor_running = False

        # 카메라 중지
        self.stop_camera = True
        for cam_num, cam_info in self.data_cameras.items():
            try:
                cam_info['cap'].release()
            except:
                pass
        self.data_cameras.clear()

        # ROS 프로세스 정리
        if hasattr(self, 'ros_process') and self.ros_process:
            try:
                import signal
                os.killpg(os.getpgid(self.ros_process.pid), signal.SIGTERM)
            except:
                pass
            self._stop_lidar_motor()
            self._kill_ros_processes()

        # Real 로봇 연결 해제
        if self.real_connected:
            self.disconnect_real()

        # Sim 연결 해제
        if self.sim_connected:
            self.disconnect_sim()

        print("리소스 정리 완료")

    def create_wheel_slider_panel(self, parent):
        """바퀴 제어 5방향 화살표 버튼 패널 (키보드 지원 + 동시 누름)"""
        # 안내
        info_label = ttk.Label(
            parent,
            text="카터 바퀴 제어 (화살표 동시 누름 가능)",
            font=("Noto Sans CJK KR", 11, "bold"),
            foreground="blue"
        )
        info_label.pack(pady=5)

        # 속도 설정
        speed_frame = ttk.Frame(parent)
        speed_frame.pack(fill=tk.X, pady=5)

        ttk.Label(speed_frame, text="바퀴 속도:", font=("Noto Sans CJK KR", 10, "bold")).pack(side=tk.LEFT, padx=5)
        speed_entry = ttk.Entry(
            speed_frame,
            textvariable=self.wheel_speed,
            width=8,
            font=("Noto Sans CJK KR", 10)
        )
        speed_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(speed_frame, text="rad/s", font=("Noto Sans CJK KR", 12)).pack(side=tk.LEFT, padx=2)

        # 화살표 제어 버튼 (5방향)
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=5)

        # 위 화살표 (전진)
        self.btn_up = tk.Button(
            button_frame,
            text="^",
            font=("Noto Sans CJK KR", 11, "bold"),
            bg="black",
            fg="white",
            width=2,
            height=1
        )
        self.btn_up.grid(row=0, column=1, padx=2, pady=2)
        self.btn_up.bind('<ButtonPress-1>', lambda e: self.on_btn_press('up'))
        self.btn_up.bind('<ButtonRelease-1>', lambda e: self.on_btn_release('up'))

        # 왼쪽 화살표 (반시계 회전)
        self.btn_left = tk.Button(
            button_frame,
            text="<",
            font=("Noto Sans CJK KR", 11, "bold"),
            bg="black",
            fg="white",
            width=2,
            height=1
        )
        self.btn_left.grid(row=1, column=0, padx=2, pady=2)
        self.btn_left.bind('<ButtonPress-1>', lambda e: self.on_btn_press('left'))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self.on_btn_release('left'))

        # 정지 버튼 (가운데)
        self.btn_stop = tk.Button(
            button_frame,
            text="[]",
            font=("Noto Sans CJK KR", 11, "bold"),
            bg="black",
            fg="white",
            width=2,
            height=1,
            command=self.wheel_stop
        )
        self.btn_stop.grid(row=1, column=1, padx=2, pady=2)

        # 오른쪽 화살표 (시계 회전)
        self.btn_right = tk.Button(
            button_frame,
            text=">",
            font=("Noto Sans CJK KR", 11, "bold"),
            bg="black",
            fg="white",
            width=2,
            height=1
        )
        self.btn_right.grid(row=1, column=2, padx=2, pady=2)
        self.btn_right.bind('<ButtonPress-1>', lambda e: self.on_btn_press('right'))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self.on_btn_release('right'))

        # 아래 화살표 (후진)
        self.btn_down = tk.Button(
            button_frame,
            text="v",
            font=("Noto Sans CJK KR", 11, "bold"),
            bg="black",
            fg="white",
            width=2,
            height=1
        )
        self.btn_down.grid(row=2, column=1, padx=2, pady=2)
        self.btn_down.bind('<ButtonPress-1>', lambda e: self.on_btn_press('down'))
        self.btn_down.bind('<ButtonRelease-1>', lambda e: self.on_btn_release('down'))

        # 안내 문구
        info_frame = ttk.Frame(parent)
        info_frame.pack(fill=tk.X, pady=5)

        ttk.Label(
            info_frame,
            text="^ 전진 | v 후진 | < 반시계 | > 시계 | [] 정지",
            font=("Noto Sans CJK KR", 12, "bold"),
            foreground="#2c3e50"
        ).pack()

        ttk.Label(
            info_frame,
            text="[i] 키보드 화살표로 제어 가능 (동시 누름 지원)",
            font=("Noto Sans CJK KR", 12),
            foreground="#7f8c8d"
        ).pack()

        # 키보드 바인딩
        self.root.bind('<Up>', lambda e: self.on_key_press('up'))
        self.root.bind('<Down>', lambda e: self.on_key_press('down'))
        self.root.bind('<Left>', lambda e: self.on_key_press('left'))
        self.root.bind('<Right>', lambda e: self.on_key_press('right'))
        self.root.bind('<KeyRelease-Up>', lambda e: self.on_key_release('up'))
        self.root.bind('<KeyRelease-Down>', lambda e: self.on_key_release('down'))
        self.root.bind('<KeyRelease-Left>', lambda e: self.on_key_release('left'))
        self.root.bind('<KeyRelease-Right>', lambda e: self.on_key_release('right'))
        self.root.bind('<space>', lambda e: self.wheel_stop())

        # ========== Lift Control Panel ==========
        lift_frame = ttk.LabelFrame(parent, text="Lift Control (Q/A Keys)", padding=10)
        lift_frame.pack(fill=tk.X, pady=5)

        # Info
        ttk.Label(
            lift_frame,
            text="Q: 올리기 (Up) | A: 내리기 (Down)",
            font=("Noto Sans CJK KR", 10, "bold"),
            foreground="#2c3e50"
        ).pack(pady=5)

        # Lift buttons
        lift_button_frame = ttk.Frame(lift_frame)
        lift_button_frame.pack(pady=5)

        self.btn_lift_up = tk.Button(
            lift_button_frame,
            text="Q ^ 올리기",
            font=("Noto Sans CJK KR", 10, "bold"),
            bg="black",
            fg="white",
            width=12,
            height=1
        )
        self.btn_lift_up.grid(row=0, column=0, padx=5)
        self.btn_lift_up.bind('<ButtonPress-1>', lambda e: self.on_lift_press('up'))
        self.btn_lift_up.bind('<ButtonRelease-1>', lambda e: self.on_lift_release('up'))

        self.btn_lift_down = tk.Button(
            lift_button_frame,
            text="A v 내리기",
            font=("Noto Sans CJK KR", 10, "bold"),
            bg="black",
            fg="white",
            width=12,
            height=1
        )
        self.btn_lift_down.grid(row=0, column=1, padx=5)
        self.btn_lift_down.bind('<ButtonPress-1>', lambda e: self.on_lift_press('down'))
        self.btn_lift_down.bind('<ButtonRelease-1>', lambda e: self.on_lift_release('down'))

        # Position display
        self.lift_position_label = ttk.Label(
            lift_frame,
            text=f"현재 높이: {self.lift_position:.2f}m",
            font=("Noto Sans CJK KR", 12),
            foreground="#7f8c8d"
        )
        self.lift_position_label.pack(pady=5)

        # Keyboard bindings for lift
        self.root.bind('<q>', lambda e: self.on_lift_key_press('up'))
        self.root.bind('<a>', lambda e: self.on_lift_key_press('down'))
        self.root.bind('<KeyRelease-q>', lambda e: self.on_lift_key_release('up'))
        self.root.bind('<KeyRelease-a>', lambda e: self.on_lift_key_release('down'))

        # ========== Xbox Controller Panel ==========
        xbox_frame = ttk.LabelFrame(parent, text="Xbox Controller", padding=10)
        xbox_frame.pack(fill=tk.X, pady=5)

        # Xbox controller status
        self.xbox_status_label = ttk.Label(
            xbox_frame,
            text="[i] Xbox 패드 연결 대기중...",
            font=("Noto Sans CJK KR", 12),
            foreground="gray"
        )
        self.xbox_status_label.pack(pady=5)

        # Xbox button display
        xbox_display_frame = tk.Frame(xbox_frame, bg="#2c3e50")
        xbox_display_frame.pack(pady=10)

        # Left side - D-pad and Left stick
        left_side = tk.Frame(xbox_display_frame, bg="#2c3e50")
        left_side.grid(row=0, column=0, padx=20)

        # D-pad
        dpad_label = tk.Label(left_side, text="D-Pad", font=("Noto Sans CJK KR", 10), fg="white", bg="#2c3e50")
        dpad_label.pack()
        dpad_frame = tk.Frame(left_side, bg="#2c3e50")
        dpad_frame.pack(pady=5)

        self.xbox_dpad_up = tk.Label(dpad_frame, text="▲", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=3)
        self.xbox_dpad_up.grid(row=0, column=1, padx=1, pady=1)

        self.xbox_dpad_left = tk.Label(dpad_frame, text="◀", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=3)
        self.xbox_dpad_left.grid(row=1, column=0, padx=1, pady=1)

        self.xbox_dpad_down = tk.Label(dpad_frame, text="▼", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=3)
        self.xbox_dpad_down.grid(row=1, column=1, padx=1, pady=1)

        self.xbox_dpad_right = tk.Label(dpad_frame, text="▶", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=3)
        self.xbox_dpad_right.grid(row=1, column=2, padx=1, pady=1)

        # Center - Start, Select, Xbox button
        center_side = tk.Frame(xbox_display_frame, bg="#2c3e50")
        center_side.grid(row=0, column=1, padx=20)

        center_buttons = tk.Frame(center_side, bg="#2c3e50")
        center_buttons.pack()

        self.xbox_select = tk.Label(center_buttons, text="◻", font=("Noto Sans CJK KR", 10), bg="gray20", fg="gray50", width=4)
        self.xbox_select.grid(row=0, column=0, padx=5)

        self.xbox_xbox_btn = tk.Label(center_buttons, text="X", font=("Noto Sans CJK KR", 10, "bold"), bg="gray20", fg="gray50", width=4)
        self.xbox_xbox_btn.grid(row=0, column=1, padx=5)

        self.xbox_start = tk.Label(center_buttons, text="≡", font=("Noto Sans CJK KR", 10), bg="gray20", fg="gray50", width=4)
        self.xbox_start.grid(row=0, column=2, padx=5)

        # Right side - A/B/X/Y buttons
        right_side = tk.Frame(xbox_display_frame, bg="#2c3e50")
        right_side.grid(row=0, column=2, padx=20)

        abxy_label = tk.Label(right_side, text="Buttons", font=("Noto Sans CJK KR", 10), fg="white", bg="#2c3e50")
        abxy_label.pack()
        abxy_frame = tk.Frame(right_side, bg="#2c3e50")
        abxy_frame.pack(pady=5)

        self.xbox_y = tk.Label(abxy_frame, text="Y", font=("Noto Sans CJK KR", 12, "bold"), bg="gray20", fg="yellow", width=3)
        self.xbox_y.grid(row=0, column=1, padx=1, pady=1)

        self.xbox_x = tk.Label(abxy_frame, text="X", font=("Noto Sans CJK KR", 12, "bold"), bg="gray20", fg="blue", width=3)
        self.xbox_x.grid(row=1, column=0, padx=1, pady=1)

        self.xbox_b = tk.Label(abxy_frame, text="B", font=("Noto Sans CJK KR", 12, "bold"), bg="gray20", fg="red", width=3)
        self.xbox_b.grid(row=1, column=2, padx=1, pady=1)

        self.xbox_a = tk.Label(abxy_frame, text="A", font=("Noto Sans CJK KR", 12, "bold"), bg="gray20", fg="green", width=3)
        self.xbox_a.grid(row=2, column=1, padx=1, pady=1)

        # Shoulders and triggers
        shoulder_frame = tk.Frame(xbox_frame, bg="#34495e")
        shoulder_frame.pack(fill=tk.X, pady=5)

        self.xbox_lb = tk.Label(shoulder_frame, text="LB", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=6)
        self.xbox_lb.pack(side=tk.LEFT, padx=10)

        self.xbox_lt = tk.Label(shoulder_frame, text="LT", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=6)
        self.xbox_lt.pack(side=tk.LEFT, padx=10)

        self.xbox_rt = tk.Label(shoulder_frame, text="RT", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=6)
        self.xbox_rt.pack(side=tk.RIGHT, padx=10)

        self.xbox_rb = tk.Label(shoulder_frame, text="RB", font=("Noto Sans CJK KR", 12), bg="gray20", fg="gray50", width=6)
        self.xbox_rb.pack(side=tk.RIGHT, padx=10)

        # Joysticks (analog sticks)
        joystick_frame = tk.Frame(xbox_frame, bg="#34495e")
        joystick_frame.pack(fill=tk.X, pady=10)

        # Left stick
        left_stick_frame = tk.Frame(joystick_frame, bg="#2c3e50", relief=tk.SUNKEN, borderwidth=2)
        left_stick_frame.pack(side=tk.LEFT, padx=20, pady=5)

        left_stick_label = tk.Label(left_stick_frame, text="Left Stick", font=("Noto Sans CJK KR", 10), fg="white", bg="#2c3e50")
        left_stick_label.pack()

        left_stick_canvas = tk.Canvas(left_stick_frame, width=80, height=80, bg="gray30", highlightthickness=0)
        left_stick_canvas.pack(padx=5, pady=5)

        # 중심 십자선
        left_stick_canvas.create_line(40, 0, 40, 80, fill="gray50", dash=(2, 2))
        left_stick_canvas.create_line(0, 40, 80, 40, fill="gray50", dash=(2, 2))

        # 조이스틱 점 (초기 중앙)
        self.xbox_left_stick_dot = left_stick_canvas.create_oval(35, 35, 45, 45, fill="cyan", outline="white")
        self.xbox_left_stick_canvas = left_stick_canvas

        # Right stick
        right_stick_frame = tk.Frame(joystick_frame, bg="#2c3e50", relief=tk.SUNKEN, borderwidth=2)
        right_stick_frame.pack(side=tk.RIGHT, padx=20, pady=5)

        right_stick_label = tk.Label(right_stick_frame, text="Right Stick", font=("Noto Sans CJK KR", 10), fg="white", bg="#2c3e50")
        right_stick_label.pack()

        right_stick_canvas = tk.Canvas(right_stick_frame, width=80, height=80, bg="gray30", highlightthickness=0)
        right_stick_canvas.pack(padx=5, pady=5)

        # 중심 십자선
        right_stick_canvas.create_line(40, 0, 40, 80, fill="gray50", dash=(2, 2))
        right_stick_canvas.create_line(0, 40, 80, 40, fill="gray50", dash=(2, 2))

        # 조이스틱 점 (초기 중앙)
        self.xbox_right_stick_dot = right_stick_canvas.create_oval(35, 35, 45, 45, fill="magenta", outline="white")
        self.xbox_right_stick_canvas = right_stick_canvas

        # 조이스틱 초기값 설정 (중립 = 0)
        self._left_stick_x = 0
        self._left_stick_y = 0
        self._right_stick_x = 0
        self._right_stick_y = 0

        # Xbox 자세 저장/불러오기 안내
        xbox_pose_frame = tk.Frame(xbox_frame, bg="#34495e")
        xbox_pose_frame.pack(fill=tk.X, pady=10)

        pose_info_label = tk.Label(
            xbox_pose_frame,
            text="슬롯 클릭 -> Start 누른 상태로 A/B/X/Y 클릭하여 버튼 할당",
            font=("Noto Sans CJK KR", 12, "bold"),
            fg="yellow",
            bg="#34495e"
        )
        pose_info_label.pack()

        # Xbox 저장된 자세 슬롯 (최대 9개)
        self.xbox_saved_poses = {}  # {slot_number: {servo_id: position}}
        self.xbox_pose_slot_buttons = []  # 버튼으로 변경

        # 슬롯을 3행 3열로 배치 (총 9개)
        slots_container = tk.Frame(xbox_pose_frame, bg="#34495e")
        slots_container.pack(pady=5)

        for row in range(3):
            row_frame = tk.Frame(slots_container, bg="#34495e")
            row_frame.pack(pady=2)

            for col in range(3):
                slot_num = row * 3 + col

                slot_btn = tk.Button(
                    row_frame,
                    text=f"슬롯 {slot_num+1}\n비어있음",
                    font=("Noto Sans CJK KR", 12),
                    fg="gray",
                    bg="#2c3e50",
                    width=15,
                    height=2,
                    relief=tk.RAISED,
                    command=lambda s=slot_num: self.select_xbox_slot(s)
                )
                # 더블클릭으로 현재 자세 저장
                slot_btn.bind("<Double-Button-1>", lambda e, s=slot_num: self.save_pose_to_slot_click(s))
                slot_btn.pack(side=tk.LEFT, padx=3)
                self.xbox_pose_slot_buttons.append(slot_btn)

        # 현재 선택된 슬롯
        self.xbox_current_slot = 0

        # 버튼별 할당된 슬롯 {button_code: slot_number}
        self.xbox_button_slot_mapping = {}

        # 할당 대기 중 플래그
        self.xbox_waiting_for_button_assignment = False

        # 중복 키 확인 대기 플래그
        self.xbox_waiting_for_conflict_response = False
        self.xbox_conflict_button_code = None
        self.xbox_conflict_old_slot = None
        self.xbox_pending_button_code = None  # 할당 대기 중인 버튼 코드

        # 저장 파일 경로 (GUI와 같은 디렉토리)
        import os
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.xbox_pose_save_file = os.path.join(script_dir, "xbox_poses.json")

        # 시작 시 저장된 포즈 불러오기
        self.load_xbox_poses_from_file()

        # Xbox button references for easy access
        self.xbox_buttons = {
            'BTN_NORTH': self.xbox_x,      # X (북쪽)
            'BTN_SOUTH': self.xbox_a,      # A (남쪽)
            'BTN_EAST': self.xbox_b,       # B (동쪽)
            'BTN_WEST': self.xbox_y,       # Y (서쪽)
            'BTN_TL': self.xbox_lb,        # LB
            'BTN_TR': self.xbox_rb,        # RB
            'BTN_SELECT': self.xbox_select,
            'BTN_START': self.xbox_start,
            'BTN_MODE': self.xbox_xbox_btn,
            'ABS_HAT0Y_-1': self.xbox_dpad_up,
            'ABS_HAT0Y_1': self.xbox_dpad_down,
            'ABS_HAT0X_-1': self.xbox_dpad_left,
            'ABS_HAT0X_1': self.xbox_dpad_right,
            'ABS_Z': self.xbox_lt,         # LT
            'ABS_RZ': self.xbox_rt,        # RT
        }

        # Start Xbox controller monitoring thread
        self.xbox_thread_running = True
        import threading
        threading.Thread(target=self.monitor_xbox_controller, daemon=True).start()


    def on_btn_press(self, direction):
        """버튼 누름"""
        if direction == 'up':
            self.btn_up_pressed = True
            self.btn_up.config(bg="#404040")  # 어둡게
        elif direction == 'down':
            self.btn_down_pressed = True
            self.btn_down.config(bg="#404040")
        elif direction == 'left':
            self.btn_left_pressed = True
            self.btn_left.config(bg="#404040")
        elif direction == 'right':
            self.btn_right_pressed = True
            self.btn_right.config(bg="#404040")

        self.update_wheel_speed()

    def on_btn_release(self, direction):
        """버튼 릴리즈"""
        if direction == 'up':
            self.btn_up_pressed = False
            self.btn_up.config(bg="black")  # 원래 색
        elif direction == 'down':
            self.btn_down_pressed = False
            self.btn_down.config(bg="black")
        elif direction == 'left':
            self.btn_left_pressed = False
            self.btn_left.config(bg="black")
        elif direction == 'right':
            self.btn_right_pressed = False
            self.btn_right.config(bg="black")

        self.update_wheel_speed()

    def on_key_press(self, direction):
        """키보드 누름"""
        if direction == 'up' and not self.btn_up_pressed:
            self.btn_up_pressed = True
            if hasattr(self, 'btn_up'):
                self.btn_up.config(bg="#404040")
        elif direction == 'down' and not self.btn_down_pressed:
            self.btn_down_pressed = True
            if hasattr(self, 'btn_down'):
                self.btn_down.config(bg="#404040")
        elif direction == 'left' and not self.btn_left_pressed:
            self.btn_left_pressed = True
            if hasattr(self, 'btn_left'):
                self.btn_left.config(bg="#404040")
        elif direction == 'right' and not self.btn_right_pressed:
            self.btn_right_pressed = True
            if hasattr(self, 'btn_right'):
                self.btn_right.config(bg="#404040")

        self.update_wheel_speed()

    def on_key_release(self, direction):
        """키보드 릴리즈"""
        if direction == 'up':
            self.btn_up_pressed = False
            if hasattr(self, 'btn_up'):
                self.btn_up.config(bg="black")
        elif direction == 'down':
            self.btn_down_pressed = False
            if hasattr(self, 'btn_down'):
                self.btn_down.config(bg="black")
        elif direction == 'left':
            self.btn_left_pressed = False
            if hasattr(self, 'btn_left'):
                self.btn_left.config(bg="black")
        elif direction == 'right':
            self.btn_right_pressed = False
            if hasattr(self, 'btn_right'):
                self.btn_right.config(bg="black")

        self.update_wheel_speed()

    def update_wheel_speed(self):
        """버튼 상태에 따라 바퀴 속도 계산 및 전송"""
        speed = self.wheel_speed.get()
        left_vel = 0.0
        right_vel = 0.0

        # 각 버튼의 기여도 계산
        if self.btn_up_pressed:
            left_vel += speed
            right_vel += speed
        if self.btn_down_pressed:
            left_vel -= speed
            right_vel -= speed
        if self.btn_left_pressed:
            left_vel -= speed
            right_vel += speed
        if self.btn_right_pressed:
            left_vel += speed
            right_vel -= speed

        # 명령 전송
        self.send_wheel_command(left_vel, right_vel)

        # 로그
        if self.sim_connected and (left_vel != 0 or right_vel != 0):
            direction_str = []
            if self.btn_up_pressed:
                direction_str.append("^")
            if self.btn_down_pressed:
                direction_str.append("v")
            if self.btn_left_pressed:
                direction_str.append("<")
            if self.btn_right_pressed:
                direction_str.append(">")

            self.log_real(f"[Wheel] {' '.join(direction_str)} L={left_vel:.1f}, R={right_vel:.1f}")

    def wheel_stop(self):
        """바퀴 정지"""
        self.btn_up_pressed = False
        self.btn_down_pressed = False
        self.btn_left_pressed = False
        self.btn_right_pressed = False


        # 버튼 색상 복원
        if hasattr(self, 'btn_up'):
            self.btn_up.config(bg="black")
        if hasattr(self, 'btn_down'):
            self.btn_down.config(bg="black")
        if hasattr(self, 'btn_left'):
            self.btn_left.config(bg="black")
        if hasattr(self, 'btn_right'):
            self.btn_right.config(bg="black")

        self.send_wheel_command(0, 0)
        if self.sim_connected:
            self.log_real("[Wheel] [] 정지")

    def on_lift_press(self, direction):
        """리프트 버튼 누름"""
        if direction == 'up':
            self.lift_q_pressed = True
            if hasattr(self, 'btn_lift_up'):
                self.btn_lift_up.config(bg="#404040")
        elif direction == 'down':
            self.lift_a_pressed = True
            if hasattr(self, 'btn_lift_down'):
                self.btn_lift_down.config(bg="#404040")

        # 이미 업데이트 루프가 돌고 있으면 새로 시작 안 함
        if not self.lift_update_running:
            self.update_lift_position()

    def on_lift_release(self, direction):
        """리프트 버튼 릴리즈"""
        if direction == 'up':
            self.lift_q_pressed = False
            if hasattr(self, 'btn_lift_up'):
                self.btn_lift_up.config(bg="black")
        elif direction == 'down':
            self.lift_a_pressed = False
            if hasattr(self, 'btn_lift_down'):
                self.btn_lift_down.config(bg="black")

        self.update_lift_position()

    def on_lift_key_press(self, direction):
        """리프트 키 누름 (0.1초 디바운스)"""
        # 0.05초 이내 재입력 무시
        now = time.time()
        if now - self.lift_last_key_time < 0.05:
            return
        self.lift_last_key_time = now

        if direction == 'up' and not self.lift_q_pressed:
            self.lift_q_pressed = True
            if hasattr(self, 'btn_lift_up'):
                self.btn_lift_up.config(bg="#404040")
        elif direction == 'down' and not self.lift_a_pressed:
            self.lift_a_pressed = True
            if hasattr(self, 'btn_lift_down'):
                self.btn_lift_down.config(bg="#404040")

        # 이미 업데이트 루프가 돌고 있으면 새로 시작 안 함
        if not self.lift_update_running:
            self.update_lift_position()

    def on_lift_key_release(self, direction):
        """리프트 키 릴리즈"""
        if direction == 'up':
            self.lift_q_pressed = False
            if hasattr(self, 'btn_lift_up'):
                self.btn_lift_up.config(bg="black")
        elif direction == 'down':
            self.lift_a_pressed = False
            if hasattr(self, 'btn_lift_down'):
                self.btn_lift_down.config(bg="black")

        self.update_lift_position()

    def update_lift_position(self):
        """리프트 위치 업데이트"""
        # 키가 안 눌려있으면 루프 종료
        if not self.lift_q_pressed and not self.lift_a_pressed:
            self.lift_update_running = False
            return

        # 이미 루프가 실행 중이면 중복 실행 방지
        if not self.lift_update_running:
            self.lift_update_running = True

        dt = 0.05  # 50ms per update

        if self.lift_q_pressed:
            # Move up (towards 0)
            self.lift_position += self.lift_speed * dt
            if self.lift_position > self.lift_max:
                self.lift_position = self.lift_max
        elif self.lift_a_pressed:
            # Move down (towards -0.5)
            self.lift_position -= self.lift_speed * dt
            if self.lift_position < self.lift_min:
                self.lift_position = self.lift_min

        # Update label
        if hasattr(self, 'lift_position_label'):
            self.lift_position_label.config(text=f"현재 높이: {self.lift_position:.2f}m")

        # Send command
        self.send_lift_command(self.lift_position)

        # Schedule next update if still pressing
        if self.lift_q_pressed or self.lift_a_pressed:
            self.root.after(50, self.update_lift_position)
        else:
            self.lift_update_running = False

    def send_lift_command(self, position):
        """리프트 위치 명령 전송"""
        try:
            data = {
                'timestamp': time.time(),
                'lift_position': position
            }

            with open(self.lift_command_file, 'w') as f:
                json.dump(data, f, indent=2)

            if self.sim_connected:
                direction = "^" if self.lift_q_pressed else "v" if self.lift_a_pressed else "[]"
                self.log_real(f"[Lift] {direction} 위치: {position:.2f}m")

        except Exception as e:
            self.log_real(f"[ERROR] 리프트 명령 전송 실패: {e}")


    def send_wheel_command(self, left_vel, right_vel):
        """바퀴 velocity 명령을 JSON 파일에 쓰기"""
        try:
            data = {
                'timestamp': time.time(),
                'left_wheel_velocity': left_vel,  # rad/s
                'right_wheel_velocity': right_vel  # rad/s
            }

            with open(self.wheel_command_file, 'w') as f:
                json.dump(data, f, indent=2)

            print(f"[DEBUG] Wheel command sent: L={left_vel:.2f}, R={right_vel:.2f}")
            print(f"[DEBUG] File written: {self.wheel_command_file}")
            self.debug_log(f"[WHEEL] L={left_vel:.2f}, R={right_vel:.2f}")

        except Exception as e:
            print(f"[ERROR] send_wheel_command failed: {e}")
            self.log_real(f"[ERROR] 바퀴 명령 전송 실패: {e}")
            import traceback
            traceback.print_exc()

    def monitor_xbox_controller(self):
        """Xbox 컨트롤러 모니터링 (별도 스레드) - 자동으로 Xbox 찾기"""
        import struct
        import time
        import select
        import subprocess

        # Xbox 컨트롤러 자동 감지
        event_path = None
        try:
            result = subprocess.run(['grep', '-l', 'Xbox', '/proc/bus/input/devices'],
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Xbox 발견, Handlers 줄에서 event 번호 추출
                devices_content = subprocess.run(['cat', '/proc/bus/input/devices'],
                                               capture_output=True, text=True, timeout=2)
                lines = devices_content.stdout.split('\n')
                for i, line in enumerate(lines):
                    if 'Xbox' in line:
                        # 다음 몇 줄에서 Handlers 찾기
                        for j in range(i, min(i+10, len(lines))):
                            if 'Handlers' in lines[j]:
                                # event15 같은 패턴 추출
                                import re
                                match = re.search(r'event(\d+)', lines[j])
                                if match:
                                    event_path = f"/dev/input/event{match.group(1)}"
                                    break
                        break
        except Exception as e:
            self.log_real(f"[Xbox] 자동 감지 실패: {e}")

        # 찾지 못하면 기본값
        if not event_path:
            event_path = "/dev/input/event17"
            self.log_real(f"[Xbox] 기본 경로 사용: {event_path}")

        # 버튼 코드 매핑 (리눅스 input event codes)
        BUTTON_CODES = {
            304: 'BTN_SOUTH',   # A
            305: 'BTN_EAST',    # B
            307: 'BTN_NORTH',   # X
            308: 'BTN_WEST',    # Y
            310: 'BTN_TL',      # LB
            311: 'BTN_TR',      # RB
            314: 'BTN_SELECT',  # Select
            315: 'BTN_START',   # Start
            316: 'BTN_MODE',    # Xbox button
        }

        DPAD_CODES = {
            16: 'ABS_HAT0X',  # 좌우: -1=왼쪽, 1=오른쪽
            17: 'ABS_HAT0Y',  # 상하: -1=위, 1=아래
        }

        TRIGGER_CODES = {
            2: 'ABS_Z',   # LT
            5: 'ABS_RZ',  # RT
        }

        try:
            self.log_real(f"[Xbox] {event_path} 열기 시도...")
            self.xbox_status_label.config(text="[...] Xbox 패드 연결 중...", foreground="yellow")

            with open(event_path, "rb") as f:
                self.log_real(f"[Xbox] [V] Xbox 컨트롤러 연결됨!")
                self.xbox_status_label.config(text="[O] Xbox 패드 연결됨!", foreground="green")

                event_count = 0
                last_log_time = time.time()

                # 버튼 상태 추적 (콤보 키 감지용)
                button_states = {}  # {code: is_pressed}

                # 트리거 상태 추적 (슬롯 할당용)
                trigger_states = {9: False, 10: False}  # {code: is_pressed}
                TRIGGER_THRESHOLD = 100  # 트리거 눌림 감지 임계값

                # D-pad 상태 추적 (가상 코드: 2016=UP, 2017=DOWN, 2018=LEFT, 2019=RIGHT)
                dpad_states = {2016: False, 2017: False, 2018: False, 2019: False}

                # 그리퍼 버튼 상태 추적
                gripper_button_states = {310: False, 311: False}  # LB, RB

                while self.xbox_thread_running:
                    # 0.001초 타임아웃으로 이벤트 대기 (더 빠른 반응)
                    ready, _, _ = select.select([f], [], [], 0.001)

                    # 먼저 모든 대기 중인 이벤트를 처리 (버퍼 비우기)
                    events_processed = 0
                    while ready and events_processed < 50:  # 최대 50개 이벤트 한번에 처리
                        events_processed += 1
                        # input_event 구조체 읽기 (24 bytes)
                        data = f.read(24)
                        if len(data) != 24:
                            continue

                        # 구조체 파싱: tv_sec, tv_usec, type, code, value
                        _, _, ev_type, ev_code, ev_value = struct.unpack('llHHi', data)

                        event_count += 1

                        # 1초마다 한 번씩 로그 (너무 많은 로그 방지)
                        if time.time() - last_log_time >= 1.0:
                            self.log_real(f"[Xbox] 이벤트 수신 중... (총 {event_count}개)")
                            last_log_time = time.time()

                        # EV_KEY (타입 1): 버튼 이벤트
                        if ev_type == 1 and ev_code in BUTTON_CODES:
                            button_name = BUTTON_CODES[ev_code]
                            is_pressed = (ev_value == 1)
                            is_released = (ev_value == 0)

                            # 버튼 상태 업데이트 (눌림과 뗌 모두)
                            if is_pressed:
                                button_states[ev_code] = True
                            elif is_released:
                                button_states[ev_code] = False

                            # Select 버튼 (314) = 기본 자세 리셋
                            if is_pressed and ev_code == 314:  # Select
                                self.xbox_reset_to_default_pose()
                                continue

                            # LB/RB는 그리퍼 펴기 전용 (메인 루프에서 처리)
                            if ev_code in [310, 311]:  # LB, RB
                                continue

                            # 할당 가능한 버튼들 (A, B, X, Y, Xbox)
                            # Start(315), Select(314), LB(310), RB(311)는 제외
                            assignable_buttons = [304, 305, 307, 308, 316]

                            if is_pressed and ev_code in assignable_buttons:
                                # Start 버튼이 눌려있는 경우: 현재 자세 저장 + 버튼 할당
                                if button_states.get(315, False):
                                    # 중복 체크
                                    conflict = self.xbox_check_button_conflict(ev_code)
                                    if conflict is None:  # 대기 중
                                        self.xbox_pending_button_code = ev_code
                                        continue
                                    elif not conflict:  # 충돌 있고 사용자가 취소
                                        continue

                                    # 충돌 없거나 사용자 승인 -> 저장 진행
                                    self.xbox_save_current_pose_to_slot()
                                    self.xbox_button_slot_mapping[ev_code] = self.xbox_current_slot
                                    self.save_xbox_poses_to_file()

                                    btn_names = {
                                        304: 'A', 305: 'B', 307: 'X', 308: 'Y',
                                        310: 'LB', 311: 'RB', 316: 'Xbox',
                                        1009: 'RT', 1010: 'LT',
                                        2016: 'UP', 2017: 'DOWN', 2018: 'LEFT', 2019: 'RIGHT'
                                    }
                                    btn_name = btn_names[ev_code]

                                    # 슬롯 버튼 텍스트 업데이트 (저장된 서보 개수 포함)
                                    if self.xbox_current_slot in self.xbox_saved_poses:
                                        servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                        text = f"슬롯 {self.xbox_current_slot+1}\n[{btn_name}] {servo_count}개"

                                        self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                            self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))

                                    self.log_real(f"[Xbox] {btn_name} 버튼 -> 슬롯 {self.xbox_current_slot+1} 저장 및 할당")
                                    continue

                                # Start 버튼 안 눌려있고, 할당된 슬롯이 있는 경우: 자세 불러오기
                                elif ev_code in self.xbox_button_slot_mapping:
                                    slot_num = self.xbox_button_slot_mapping[ev_code]
                                    self.xbox_load_pose_from_slot(slot_num)
                                    continue

                            # 일반 버튼 시각 효과
                            if button_name in self.xbox_buttons:
                                button_widget = self.xbox_buttons[button_name]
                                if is_pressed:
                                    button_widget.config(bg="white")
                                else:
                                    button_widget.config(bg="gray20")

                        # EV_ABS (타입 3): D-pad, 트리거, 조이스틱
                        elif ev_type == 3:
                            # 중복 확인 대기 중이면 X/O 입력 처리
                            if self.xbox_waiting_for_conflict_response and ev_code == 16:
                                if ev_value == 1:  # 오른쪽 (취소 - O)
                                    # 다이얼로그 닫기
                                    if hasattr(self, 'xbox_conflict_dialog') and self.xbox_conflict_dialog:
                                        self.root.after(0, self.xbox_conflict_dialog.destroy)
                                        self.xbox_conflict_dialog = None
                                    self.log_real("[Xbox] 중복 키 할당 취소됨")
                                    self.xbox_waiting_for_conflict_response = False
                                    self.xbox_pending_button_code = None
                                elif ev_value == -1:  # 왼쪽 (변경 - X)
                                    # 다이얼로그 닫기
                                    if hasattr(self, 'xbox_conflict_dialog') and self.xbox_conflict_dialog:
                                        self.root.after(0, self.xbox_conflict_dialog.destroy)
                                        self.xbox_conflict_dialog = None

                                    self.log_real("[Xbox] 이전 할당 제거 및 새 슬롯에 할당")

                                    # 이전 슬롯에서 버튼 표시 제거
                                    self.xbox_update_slot_button_after_reassign(self.xbox_conflict_old_slot, self.xbox_conflict_button_code)

                                    # 새 슬롯에 할당
                                    self.xbox_button_slot_mapping[self.xbox_pending_button_code] = self.xbox_current_slot
                                    self.xbox_save_current_pose_to_slot()
                                    self.save_xbox_poses_to_file()

                                    btn_names = {
                                        304: 'A', 305: 'B', 307: 'X', 308: 'Y',
                                        310: 'LB', 311: 'RB', 316: 'Xbox',
                                        1009: 'RT', 1010: 'LT',
                                        2016: 'UP', 2017: 'DOWN', 2018: 'LEFT', 2019: 'RIGHT'
                                    }
                                    btn_name = btn_names.get(self.xbox_pending_button_code, '?')

                                    if self.xbox_current_slot in self.xbox_saved_poses:
                                        servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                        text = f"슬롯 {self.xbox_current_slot+1}\n[{btn_name}] {servo_count}개"
                                        self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                            self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))

                                    self.xbox_waiting_for_conflict_response = False
                                    self.xbox_pending_button_code = None
                                continue

                            # D-pad
                            if ev_code == 16:  # 좌우
                                if ev_value == -1:  # 왼쪽
                                    self.xbox_dpad_left.config(bg="white")
                                    self.root.after(100, lambda: self.xbox_dpad_left.config(bg="gray20"))

                                    # 슬롯 할당 처리
                                    old_state = dpad_states[2018]
                                    dpad_states[2018] = True
                                    if not old_state:
                                        if button_states.get(315, False):  # Start 눌려있음
                                            conflict = self.xbox_check_button_conflict(2018)
                                            if conflict is None:  # 대기 중
                                                self.xbox_pending_button_code = 2018
                                            elif conflict:  # 충돌 없음
                                                self.xbox_save_current_pose_to_slot()
                                                self.xbox_button_slot_mapping[2018] = self.xbox_current_slot
                                                self.save_xbox_poses_to_file()
                                                if self.xbox_current_slot in self.xbox_saved_poses:
                                                    servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                                    text = f"슬롯 {self.xbox_current_slot+1}\n[LEFT] {servo_count}개"
                                                    self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                                        self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))
                                                self.log_real(f"[Xbox] LEFT -> 슬롯 {self.xbox_current_slot+1} 할당")
                                        elif 2018 in self.xbox_button_slot_mapping:
                                            self.xbox_load_pose_from_slot(self.xbox_button_slot_mapping[2018])

                                elif ev_value == 1:  # 오른쪽
                                    self.xbox_dpad_right.config(bg="white")
                                    self.root.after(100, lambda: self.xbox_dpad_right.config(bg="gray20"))

                                    # 슬롯 할당 처리
                                    old_state = dpad_states[2019]
                                    dpad_states[2019] = True
                                    if not old_state:
                                        if button_states.get(315, False):  # Start 눌려있음
                                            conflict = self.xbox_check_button_conflict(2019)
                                            if conflict is None:  # 대기 중
                                                self.xbox_pending_button_code = 2019
                                            elif conflict:  # 충돌 없음
                                                self.xbox_save_current_pose_to_slot()
                                                self.xbox_button_slot_mapping[2019] = self.xbox_current_slot
                                                self.save_xbox_poses_to_file()
                                                if self.xbox_current_slot in self.xbox_saved_poses:
                                                    servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                                    text = f"슬롯 {self.xbox_current_slot+1}\n[RIGHT] {servo_count}개"
                                                    self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                                        self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))
                                                self.log_real(f"[Xbox] RIGHT -> 슬롯 {self.xbox_current_slot+1} 할당")
                                        elif 2019 in self.xbox_button_slot_mapping:
                                            self.xbox_load_pose_from_slot(self.xbox_button_slot_mapping[2019])

                                elif ev_value == 0:  # 중립
                                    self.xbox_dpad_left.config(bg="gray20")
                                    self.xbox_dpad_right.config(bg="gray20")
                                    dpad_states[2018] = False
                                    dpad_states[2019] = False

                            elif ev_code == 17:  # 상하
                                if ev_value == -1:  # 위
                                    self.xbox_dpad_up.config(bg="white")
                                    self.root.after(100, lambda: self.xbox_dpad_up.config(bg="gray20"))

                                    # 슬롯 할당 처리
                                    old_state = dpad_states[2016]
                                    dpad_states[2016] = True
                                    if not old_state:
                                        if button_states.get(315, False):  # Start 눌려있음
                                            conflict = self.xbox_check_button_conflict(2016)
                                            if conflict is None:  # 대기 중
                                                self.xbox_pending_button_code = 2016
                                            elif conflict:  # 충돌 없음
                                                self.xbox_save_current_pose_to_slot()
                                                self.xbox_button_slot_mapping[2016] = self.xbox_current_slot
                                                self.save_xbox_poses_to_file()
                                                if self.xbox_current_slot in self.xbox_saved_poses:
                                                    servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                                    text = f"슬롯 {self.xbox_current_slot+1}\n[UP] {servo_count}개"
                                                    self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                                        self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))
                                                self.log_real(f"[Xbox] UP -> 슬롯 {self.xbox_current_slot+1} 할당")
                                        elif 2016 in self.xbox_button_slot_mapping:
                                            self.xbox_load_pose_from_slot(self.xbox_button_slot_mapping[2016])

                                elif ev_value == 1:  # 아래
                                    self.xbox_dpad_down.config(bg="white")
                                    self.root.after(100, lambda: self.xbox_dpad_down.config(bg="gray20"))

                                    # 슬롯 할당 처리
                                    old_state = dpad_states[2017]
                                    dpad_states[2017] = True
                                    if not old_state:
                                        if button_states.get(315, False):  # Start 눌려있음
                                            conflict = self.xbox_check_button_conflict(2017)
                                            if conflict is None:  # 대기 중
                                                self.xbox_pending_button_code = 2017
                                            elif conflict:  # 충돌 없음
                                                self.xbox_save_current_pose_to_slot()
                                                self.xbox_button_slot_mapping[2017] = self.xbox_current_slot
                                                self.save_xbox_poses_to_file()
                                                if self.xbox_current_slot in self.xbox_saved_poses:
                                                    servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
                                                    text = f"슬롯 {self.xbox_current_slot+1}\n[DOWN] {servo_count}개"
                                                    self.root.after(0, lambda t=text, s=self.xbox_current_slot:
                                                        self.xbox_pose_slot_buttons[s].config(text=t, fg="lime"))
                                                self.log_real(f"[Xbox] DOWN -> 슬롯 {self.xbox_current_slot+1} 할당")
                                        elif 2017 in self.xbox_button_slot_mapping:
                                            self.xbox_load_pose_from_slot(self.xbox_button_slot_mapping[2017])

                                elif ev_value == 0:  # 중립
                                    self.xbox_dpad_up.config(bg="gray20")
                                    self.xbox_dpad_down.config(bg="gray20")
                                    dpad_states[2016] = False
                                    dpad_states[2017] = False

                            # 왼쪽 조이스틱 (code 0: X축, code 1: Y축)
                            # 값 범위: 0 ~ 65535 (부호 없는 16비트, 중립은 32768 근처)
                            if ev_code == 0:  # Left stick X
                                # 0~65535를 -32768~32767로 변환
                                centered = ev_value - 32768

                                # Dead zone 적용 (±2000 이하는 중립)
                                if abs(centered) < 2000:
                                    self._left_stick_x = 0
                                else:
                                    self._left_stick_x = centered

                                # 캔버스 좌표로 변환 (중심 40, 범위 ±35)
                                x_pos = 40 + (self._left_stick_x / 32768.0) * 35
                                y_pos = 40 + (self._left_stick_y / 32768.0) * 35
                                x_pos = max(5, min(75, x_pos))
                                y_pos = max(5, min(75, y_pos))
                                self.xbox_left_stick_canvas.coords(self.xbox_left_stick_dot, x_pos-5, y_pos-5, x_pos+5, y_pos+5)

                            elif ev_code == 1:  # Left stick Y
                                centered = ev_value - 32768

                                if abs(centered) < 2000:
                                    self._left_stick_y = 0
                                else:
                                    self._left_stick_y = centered

                                x_pos = 40 + (self._left_stick_x / 32768.0) * 35
                                y_pos = 40 + (self._left_stick_y / 32768.0) * 35
                                x_pos = max(5, min(75, x_pos))
                                y_pos = max(5, min(75, y_pos))
                                self.xbox_left_stick_canvas.coords(self.xbox_left_stick_dot, x_pos-5, y_pos-5, x_pos+5, y_pos+5)

                            # 오른쪽 조이스틱 (code 2: X축, code 5: Y축)
                            # Xbox Wireless Controller에서 Z/RZ가 오른쪽 스틱으로 매핑됨
                            # 범위: 0 ~ 65535 (부호 없는 16비트, 중립은 32768 근처)
                            elif ev_code == 2:  # Right stick X (ABS_Z)
                                # 0~65535를 -32768~32767로 변환
                                centered = ev_value - 32768

                                # Dead zone 적용 (±2000 이하는 중립)
                                if abs(centered) < 2000:
                                    self._right_stick_x = 0
                                else:
                                    self._right_stick_x = centered

                                x_pos = 40 + (self._right_stick_x / 32768.0) * 35
                                y_pos = 40 + (self._right_stick_y / 32768.0) * 35
                                x_pos = max(5, min(75, x_pos))
                                y_pos = max(5, min(75, y_pos))
                                self.xbox_right_stick_canvas.coords(self.xbox_right_stick_dot, x_pos-5, y_pos-5, x_pos+5, y_pos+5)

                            elif ev_code == 5:  # Right stick Y (ABS_RZ)
                                centered = ev_value - 32768

                                if abs(centered) < 2000:
                                    self._right_stick_y = 0
                                else:
                                    self._right_stick_y = centered

                                x_pos = 40 + (self._right_stick_x / 32768.0) * 35
                                y_pos = 40 + (self._right_stick_y / 32768.0) * 35
                                x_pos = max(5, min(75, x_pos))
                                y_pos = max(5, min(75, y_pos))
                                self.xbox_right_stick_canvas.coords(self.xbox_right_stick_dot, x_pos-5, y_pos-5, x_pos+5, y_pos+5)

                            # 트리거 (code 9: RT, code 10: LT) - 그리퍼 제어
                            # RT = 오른손 잡기, LT = 왼손 잡기
                            elif ev_code == 9:  # RT (ABS_BRAKE) - 오른손 잡기
                                is_pressed = ev_value > TRIGGER_THRESHOLD
                                trigger_states[9] = is_pressed

                                # 시각 효과 (그리퍼 제어는 메인 루프에서)
                                if is_pressed:
                                    self.xbox_rt.config(bg="white")
                                else:
                                    self.xbox_rt.config(bg="gray20")

                            elif ev_code == 10:  # LT (ABS_GAS) - 왼손 잡기
                                is_pressed = ev_value > TRIGGER_THRESHOLD
                                trigger_states[10] = is_pressed

                                # 시각 효과 (그리퍼 제어는 메인 루프에서)
                                if is_pressed:
                                    self.xbox_lt.config(bg="white")
                                else:
                                    self.xbox_lt.config(bg="gray20")

                        # 버퍼를 다 비우고 select를 다시 체크해서 더 읽을 게 있으면 계속
                        ready, _, _ = select.select([f], [], [], 0)

                    # 모든 이벤트를 처리한 후에 그리퍼 제어 (최신 버튼 상태로)
                    if button_states.get(310, False):  # LB - 왼손 펴기
                        self.xbox_control_gripper(31, 28, 15, close=False)
                    if button_states.get(311, False):  # RB - 오른손 펴기
                        self.xbox_control_gripper(41, 32, 15, close=False)
                    if trigger_states.get(9, False):  # RT - 오른손 잡기
                        self.xbox_control_gripper(41, 32, 15, close=True)
                    if trigger_states.get(10, False):  # LT - 왼손 잡기
                        self.xbox_control_gripper(31, 28, 15, close=True)

                self.log_real(f"[Xbox] 모니터링 종료 (총 {event_count}개 이벤트 처리)")

        except PermissionError:
            self.log_real(f"[Xbox] 권한 오류: {event_path}를 읽을 수 없습니다.")
            self.log_real("[Xbox] 해결: sudo usermod -a -G input $USER && 로그아웃 후 재로그인")
            self.xbox_status_label.config(text="[X] 권한 없음", foreground="red")
        except FileNotFoundError:
            self.log_real(f"[Xbox] Xbox 컨트롤러를 찾을 수 없습니다: {event_path}")
            self.log_real("[Xbox] 블루투스로 Xbox 패드를 연결해주세요.")
            self.xbox_status_label.config(text="[X] Xbox 패드 없음", foreground="red")
        except Exception as e:
            self.log_real(f"[Xbox] 오류 발생: {e}")
            import traceback
            self.log_real(f"[Xbox] {traceback.format_exc()}")
            self.xbox_status_label.config(text="[X] 오류 발생", foreground="red")

    def select_xbox_slot(self, slot_num):
        """슬롯 버튼 클릭 시 선택"""
        self.xbox_current_slot = slot_num

        # 모든 슬롯 버튼 배경색 리셋
        for i, btn in enumerate(self.xbox_pose_slot_buttons):
            if i == self.xbox_current_slot:
                btn.config(bg="#4a6fa5", relief=tk.SUNKEN)  # 선택된 슬롯 강조
            else:
                btn.config(bg="#2c3e50", relief=tk.RAISED)

        self.log_real(f"[Xbox] 슬롯 {self.xbox_current_slot+1} 선택됨")

    def save_pose_to_slot_click(self, slot_num):
        """슬롯 버튼 더블클릭 시 현재 자세 저장"""
        # 먼저 슬롯 선택
        self.select_xbox_slot(slot_num)
        # 현재 자세 저장
        self.xbox_save_current_pose_to_slot()

    def xbox_check_button_conflict(self, new_button_code):
        """버튼 중복 체크 및 사용자 확인"""
        btn_names = {
            304: 'A', 305: 'B', 307: 'X', 308: 'Y', 316: 'Xbox',
            2016: 'UP', 2017: 'DOWN', 2018: 'LEFT', 2019: 'RIGHT'
        }

        # 이미 할당된 버튼인지 확인
        if new_button_code in self.xbox_button_slot_mapping:
            old_slot = self.xbox_button_slot_mapping[new_button_code]
            btn_name = btn_names.get(new_button_code, '?')

            # 같은 슬롯에 재할당하는 경우는 OK
            if old_slot == self.xbox_current_slot:
                return True

            # GUI에 확인 다이얼로그 표시
            self.xbox_show_conflict_dialog(btn_name, old_slot, new_button_code)

            # 사용자가 X/O 버튼으로 선택할 때까지 대기하는 플래그 설정
            self.xbox_waiting_for_conflict_response = True
            self.xbox_conflict_button_code = new_button_code
            self.xbox_conflict_old_slot = old_slot

            return None  # 대기 상태

        return True  # 충돌 없음

    def xbox_show_conflict_dialog(self, btn_name, old_slot, button_code):
        """중복 키 할당 확인 다이얼로그 표시"""
        # 기존 다이얼로그가 있으면 제거
        if hasattr(self, 'xbox_conflict_dialog') and self.xbox_conflict_dialog:
            self.xbox_conflict_dialog.destroy()

        # 다이얼로그 생성
        dialog = tk.Toplevel(self.root)
        dialog.title("버튼 중복 할당")
        dialog.geometry("520x260")
        dialog.configure(bg="#2c3e50")
        dialog.transient(self.root)
        dialog.grab_set()

        # 메시지
        msg_frame = tk.Frame(dialog, bg="#2c3e50")
        msg_frame.pack(pady=20, padx=20)

        tk.Label(
            msg_frame,
            text=f"[{btn_name}] 버튼이 이미 슬롯 {old_slot+1}에\n할당되어 있습니다.",
            font=("Noto Sans CJK KR", 12, "bold"),
            bg="#2c3e50",
            fg="yellow"
        ).pack()

        tk.Label(
            msg_frame,
            text=f"슬롯 {self.xbox_current_slot+1}로 변경하시겠습니까?",
            font=("Noto Sans CJK KR", 11),
            bg="#2c3e50",
            fg="white"
        ).pack(pady=10)

        # 버튼 프레임
        btn_frame = tk.Frame(dialog, bg="#2c3e50")
        btn_frame.pack(pady=10)

        tk.Button(
            btn_frame,
            text="<- X\n변경",
            font=("Noto Sans CJK KR", 12, "bold"),
            bg="#27ae60",
            fg="white",
            width=10,
            height=3,
            command=lambda: self.xbox_conflict_response_yes(dialog)
        ).pack(side=tk.LEFT, padx=10)

        tk.Button(
            btn_frame,
            text="O ->\n취소",
            font=("Noto Sans CJK KR", 12, "bold"),
            bg="#e74c3c",
            fg="white",
            width=10,
            height=3,
            command=lambda: self.xbox_conflict_response_no(dialog)
        ).pack(side=tk.LEFT, padx=10)

        tk.Label(
            dialog,
            text="또는 Xbox 패드의 <- (변경) / -> (취소) 버튼을 누르세요",
            font=("Noto Sans CJK KR", 12),
            bg="#2c3e50",
            fg="gray"
        ).pack(pady=5)

        self.xbox_conflict_dialog = dialog
        self.log_real(f"[Xbox] 경고: {btn_name} 버튼 중복 - 슬롯 {old_slot+1}에서 슬롯 {self.xbox_current_slot+1}로 변경 확인 대기")

    def xbox_conflict_response_yes(self, dialog):
        """중복 키 할당 - 변경 승인"""
        dialog.destroy()
        self.xbox_conflict_dialog = None

        # 이전 슬롯에서 버튼 표시 제거
        self.xbox_update_slot_button_after_reassign(self.xbox_conflict_old_slot, self.xbox_conflict_button_code)

        # 새 슬롯에 할당
        self.log_real("[Xbox] 이전 할당 제거 및 새 슬롯에 할당")
        self.xbox_button_slot_mapping[self.xbox_pending_button_code] = self.xbox_current_slot
        self.xbox_save_current_pose_to_slot()
        self.save_xbox_poses_to_file()

        btn_names = {
            304: 'A', 305: 'B', 307: 'X', 308: 'Y',
            310: 'LB', 311: 'RB', 316: 'Xbox',
            1009: 'RT', 1010: 'LT',
            2016: 'UP', 2017: 'DOWN', 2018: 'LEFT', 2019: 'RIGHT'
        }
        btn_name = btn_names.get(self.xbox_pending_button_code, '?')

        if self.xbox_current_slot in self.xbox_saved_poses:
            servo_count = len(self.xbox_saved_poses[self.xbox_current_slot])
            text = f"슬롯 {self.xbox_current_slot+1}\n[{btn_name}] {servo_count}개"
            self.xbox_pose_slot_buttons[self.xbox_current_slot].config(text=text, fg="lime")

        self.xbox_waiting_for_conflict_response = False
        self.xbox_pending_button_code = None

    def xbox_conflict_response_no(self, dialog):
        """중복 키 할당 - 취소"""
        dialog.destroy()
        self.xbox_conflict_dialog = None
        self.log_real("[Xbox] 중복 키 할당 취소됨")
        self.xbox_waiting_for_conflict_response = False
        self.xbox_pending_button_code = None

    def xbox_update_slot_button_after_reassign(self, old_slot, button_code):
        """이전 슬롯의 버튼 표시 업데이트 (키 재할당 후)"""
        # 이전 슬롯에 자세가 남아있는지 확인
        if old_slot in self.xbox_saved_poses:
            servo_count = len(self.xbox_saved_poses[old_slot])
            text = f"슬롯 {old_slot+1}\n{servo_count}개 서보"
            self.xbox_pose_slot_buttons[old_slot].config(text=text, fg="lime")
        else:
            text = f"슬롯 {old_slot+1}\n비어있음"
            self.xbox_pose_slot_buttons[old_slot].config(text=text, fg="gray")

    def xbox_save_current_pose_to_slot(self):
        """현재 자세를 선택된 슬롯에 저장"""
        try:
            # 현재 모든 서보 위치 저장
            current_pose = {}
            for servo_id in self.real_active_joints.keys():
                current_pose[servo_id] = self.real_active_joints[servo_id]['current']

            # 슬롯에 저장
            self.xbox_saved_poses[self.xbox_current_slot] = current_pose

            # 파일로 저장
            self.save_xbox_poses_to_file()

            # 슬롯 버튼 업데이트
            servo_count = len(current_pose)

            # 할당된 버튼 표시
            assigned_btn = None
            for btn_code, slot in self.xbox_button_slot_mapping.items():
                if slot == self.xbox_current_slot:
                    btn_names = {304: 'A', 305: 'B', 307: 'X', 308: 'Y'}
                    assigned_btn = btn_names.get(btn_code, '?')
                    break

            if assigned_btn:
                text = f"슬롯 {self.xbox_current_slot+1}\n[{assigned_btn}] {servo_count}개"
            else:
                text = f"슬롯 {self.xbox_current_slot+1}\n{servo_count}개 서보"

            self.xbox_pose_slot_buttons[self.xbox_current_slot].config(
                text=text,
                fg="lime"
            )

            self.log_real(f"[Xbox] 슬롯 {self.xbox_current_slot+1}에 자세 저장 ({servo_count}개 서보)")

        except Exception as e:
            self.log_real(f"[Xbox] 자세 저장 실패: {e}")

    def xbox_load_pose_from_slot(self, slot_num):
        """지정된 슬롯에서 자세 불러오기"""
        try:
            # 슬롯에 자세가 있는지 확인
            if slot_num not in self.xbox_saved_poses:
                self.log_real(f"[Xbox] 슬롯 {slot_num+1}이 비어있습니다")
                return

            # 저장된 자세 가져오기
            saved_pose = self.xbox_saved_poses[slot_num]

            # 모든 서보를 저장된 위치로 이동
            moved_count = 0
            for servo_id, position in saved_pose.items():
                # servo_id를 정수로 변환 (JSON에서 문자열로 로드될 수 있음)
                servo_id = int(servo_id)
                if servo_id in self.real_active_joints:
                    self.real_active_joints[servo_id]['current'] = position

                    # 실제 하드웨어로 전송
                    if self.real_time_enabled.get() and self.real_connected:
                        try:
                            self.send_real_servo_command(servo_id, position)
                            moved_count += 1
                        except Exception as e:
                            self.log_real(f"[Xbox] 서보 {servo_id} 이동 실패: {e}")

                    # 슬라이더 업데이트
                    if servo_id in self.real_sliders:
                        self.real_sliders[servo_id].set(position)

            self.log_real(f"[Xbox] 슬롯 {slot_num+1}에서 자세 불러옴 ({moved_count}개 서보 이동)")

        except Exception as e:
            self.log_real(f"[Xbox] 자세 불러오기 실패: {e}")

    def xbox_reset_to_default_pose(self):
        """GUI의 기본 자세 함수 호출 (GUI 버튼과 동일한 동작)"""
        try:
            self.log_real("[Xbox] Select 버튼 - 기본 자세로 이동")
            self.move_real_to_baseline()
        except Exception as e:
            self.log_real(f"[Xbox] 기본 자세 리셋 실패: {e}")

    def xbox_control_gripper(self, tip1_id, tip2_id, speed, close=True):
        """그리퍼 제어 (누르고 있으면 계속 움직임)"""
        try:
            if not self.real_time_enabled.get() or not self.real_connected:
                return

            # 현재 값 가져오기
            current_tip1 = self.real_active_joints[tip1_id]['current']
            current_tip2 = self.real_active_joints[tip2_id]['current']

            # 새 값 계산
            if close:  # 잡기 (값 증가)
                new_tip1 = min(current_tip1 + speed, 4095)
                new_tip2 = min(current_tip2 + speed, 4095)
            else:  # 펴기 (값 감소)
                new_tip1 = max(current_tip1 - speed, 0)
                new_tip2 = max(current_tip2 - speed, 0)

            # 값이 변경되었으면 서보 이동 (슬라이더 업데이트 생략 - 성능 향상)
            if new_tip1 != current_tip1:
                self.real_active_joints[tip1_id]['current'] = new_tip1
                self.send_real_servo_command(tip1_id, new_tip1)

            if new_tip2 != current_tip2:
                self.real_active_joints[tip2_id]['current'] = new_tip2
                self.send_real_servo_command(tip2_id, new_tip2)

        except Exception as e:
            self.log_real(f"[Xbox] 그리퍼 제어 오류: {e}")

    def save_xbox_poses_to_file(self):
        """Xbox 포즈 데이터를 파일로 저장"""
        try:
            import json
            data = {
                'poses': self.xbox_saved_poses,
                'button_mapping': self.xbox_button_slot_mapping
            }

            with open(self.xbox_pose_save_file, 'w') as f:
                json.dump(data, f, indent=2)

            self.log_real(f"[Xbox] 포즈 데이터 저장됨: {self.xbox_pose_save_file}")
        except Exception as e:
            self.log_real(f"[Xbox] 포즈 저장 실패: {e}")

    def load_xbox_poses_from_file(self):
        """파일에서 Xbox 포즈 데이터 불러오기"""
        try:
            import json
            import os

            if not os.path.exists(self.xbox_pose_save_file):
                self.log_real("[Xbox] 저장된 포즈 파일 없음 (새로 시작)")
                return

            with open(self.xbox_pose_save_file, 'r') as f:
                data = json.load(f)

            # 문자열 키를 정수로 변환
            self.xbox_saved_poses = {int(k): v for k, v in data.get('poses', {}).items()}

            # LT(1010), RT(1009), LB(310), RB(311) 매핑 제거 (그리퍼 전용)
            gripper_buttons = [1010, 1009, 310, 311]
            button_mapping = {int(k): v for k, v in data.get('button_mapping', {}).items()}
            self.xbox_button_slot_mapping = {k: v for k, v in button_mapping.items() if k not in gripper_buttons}

            # 슬롯 버튼 UI 업데이트
            self.root.after(100, self.update_slot_buttons_from_loaded_data)

            self.log_real(f"[Xbox] 포즈 데이터 불러옴: {len(self.xbox_saved_poses)}개 슬롯")
        except Exception as e:
            self.log_real(f"[Xbox] 포즈 불러오기 실패: {e}")

    def update_slot_buttons_from_loaded_data(self):
        """불러온 데이터로 슬롯 버튼 UI 업데이트"""
        try:
            btn_names = {
                304: 'A', 305: 'B', 307: 'X', 308: 'Y',
                316: 'Xbox',
                2016: '[UP]', 2017: '[DOWN]', 2018: '<-', 2019: '->'
            }

            for slot_num in range(9):  # 9개 슬롯
                if slot_num < len(self.xbox_pose_slot_buttons):
                    # 할당된 버튼 찾기
                    assigned_btn = None
                    for btn_code, slot in self.xbox_button_slot_mapping.items():
                        if slot == slot_num:
                            assigned_btn = btn_names.get(btn_code, '?')
                            break

                    # 슬롯에 저장된 자세가 있는지 확인
                    if slot_num in self.xbox_saved_poses:
                        servo_count = len(self.xbox_saved_poses[slot_num])
                        if assigned_btn:
                            text = f"슬롯 {slot_num+1}\n[{assigned_btn}] {servo_count}개"
                        else:
                            text = f"슬롯 {slot_num+1}\n{servo_count}개 서보"
                        self.xbox_pose_slot_buttons[slot_num].config(text=text, fg="lime")
                    elif assigned_btn:
                        text = f"슬롯 {slot_num+1}\n[{assigned_btn}] 비어있음"
                        self.xbox_pose_slot_buttons[slot_num].config(text=text, fg="gray")
        except Exception as e:
            self.log_real(f"[Xbox] 슬롯 UI 업데이트 실패: {e}")

    # ===================================================================
    # 데이터 수집 (Pi0 파인튜닝용) - 통합 녹화 기능
    # ===================================================================

    def create_data_collection_panel(self, parent):
        """데이터 수집 패널 생성"""
        # 카메라 뷰
        cam_frame = ttk.LabelFrame(parent, text="카메라 뷰", padding=5)
        cam_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # 카메라 그리드 (동적 생성)
        self.cam_grid = ttk.Frame(cam_frame)
        self.cam_grid.pack(fill=tk.BOTH, expand=True, pady=5)

        # 연결된 카메라 감지 및 UI 생성
        self.detected_cameras = self._detect_cameras()
        self._create_camera_ui()

        # 카메라 버튼
        cam_btn_frame = ttk.Frame(cam_frame)
        cam_btn_frame.pack(fill=tk.X, pady=5)

        self.data_cam_start_btn = ttk.Button(
            cam_btn_frame, text="전체 카메라 시작", command=self.start_data_camera, width=15
        )
        self.data_cam_start_btn.pack(side=tk.LEFT, padx=2)

        ttk.Button(
            cam_btn_frame, text="카메라 중지", command=self.stop_data_camera, width=12
        ).pack(side=tk.LEFT, padx=2)

        ttk.Button(
            cam_btn_frame, text="카메라 재검색", command=self._refresh_cameras, width=12
        ).pack(side=tk.LEFT, padx=2)

        # 녹화 컨트롤
        rec_frame = ttk.LabelFrame(parent, text="녹화 컨트롤", padding=5)
        rec_frame.pack(fill=tk.X, pady=5)

        # 녹화 버튼
        self.data_record_btn = tk.Button(
            rec_frame, text="[REC] Start", font=('Noto Sans CJK KR', 14, 'bold'),
            bg='#28a745', fg='white', width=20, height=2,
            command=self.toggle_data_recording
        )
        self.data_record_btn.pack(pady=10)

        # 녹화 취소 버튼
        self.data_cancel_btn = tk.Button(
            rec_frame, text="Cancel (Don't Save)", font=('Noto Sans CJK KR', 10),
            bg='#6c757d', fg='white', width=20,
            command=self.cancel_data_recording, state='disabled'
        )
        self.data_cancel_btn.pack(pady=2)

        # 이전 녹화 재생 버튼
        self.data_playback_btn = tk.Button(
            rec_frame, text="▶ Play Recording", font=('Noto Sans CJK KR', 10),
            bg='#17a2b8', fg='white', width=20,
            command=self.play_previous_recording
        )
        self.data_playback_btn.pack(pady=5)

        # 텔레옵 Play 버튼 (비디오/저장 없이 관절 연동만)
        self.teleop_play_btn = tk.Button(
            rec_frame, text="▶ Teleop Play", font=('Noto Sans CJK KR', 10, 'bold'),
            bg='#ff8c00', fg='white', width=20,
            command=self.toggle_teleop_play
        )
        self.teleop_play_btn.pack(pady=5)

        # 키보드 녹화 컨트롤 토글
        kb_frame = ttk.Frame(rec_frame)
        kb_frame.pack(fill=tk.X, pady=5)

        self.keyboard_rec_var = tk.BooleanVar(value=False)
        self.keyboard_rec_check = ttk.Checkbutton(
            kb_frame, text="키보드 녹화 (Space: 시작/저장, 방향키: 취소)",
            variable=self.keyboard_rec_var,
            command=self._toggle_keyboard_recording
        )
        self.keyboard_rec_check.pack(side=tk.LEFT)

        # 녹화 상태 표시
        status_frame = ttk.Frame(rec_frame)
        status_frame.pack(fill=tk.X, pady=5)

        ttk.Label(status_frame, text="에피소드:").pack(side=tk.LEFT, padx=2)
        self.data_episode_label = ttk.Label(status_frame, text="0", font=('Noto Sans CJK KR', 11, 'bold'))
        self.data_episode_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(status_frame, text="프레임:").pack(side=tk.LEFT, padx=10)
        self.data_frame_label = ttk.Label(status_frame, text="0", font=('Noto Sans CJK KR', 11, 'bold'))
        self.data_frame_label.pack(side=tk.LEFT, padx=5)

        # 시간 표시
        time_frame = ttk.Frame(rec_frame)
        time_frame.pack(fill=tk.X)

        ttk.Label(time_frame, text="시간:").pack(side=tk.LEFT, padx=2)
        self.data_duration_label = ttk.Label(time_frame, text="0.0초", font=('Noto Sans CJK KR', 11, 'bold'))
        self.data_duration_label.pack(side=tk.LEFT, padx=5)

        # FPS 설정
        fps_frame = ttk.Frame(rec_frame)
        fps_frame.pack(fill=tk.X, pady=5)

        ttk.Label(fps_frame, text="FPS:").pack(side=tk.LEFT, padx=2)
        self.data_fps_var = tk.StringVar(value="11")
        fps_combo = ttk.Combobox(fps_frame, textvariable=self.data_fps_var, width=5,
                                  values=['5', '10', '11', '15', '20', '30'])
        fps_combo.pack(side=tk.LEFT, padx=5)
        fps_combo.bind('<<ComboboxSelected>>', lambda e: self.update_data_fps())

        # 데이터셋 설정
        ds_frame = ttk.LabelFrame(parent, text="데이터셋 설정", padding=5)
        ds_frame.pack(fill=tk.X, pady=5)

        # 데이터셋 이름
        ttk.Label(ds_frame, text="데이터셋:").pack(anchor=tk.W)
        ttk.Entry(ds_frame, textvariable=self.current_dataset_name, width=25).pack(fill=tk.X, pady=2)

        # 태스크 선택
        ttk.Label(ds_frame, text="태스크:").pack(anchor=tk.W, pady=(5, 0))
        task_combo = ttk.Combobox(
            ds_frame, textvariable=self.current_task_name, width=23,
            values=['pick_and_place', 'fold_clothes', 'fold_towel', 'stack_objects',
                    'open_drawer', 'close_drawer', 'pour_liquid', 'wipe_surface', 'custom']
        )
        task_combo.pack(fill=tk.X, pady=2)

        # 언어 명령
        prompt_frame = ttk.LabelFrame(parent, text="언어 명령", padding=5)
        prompt_frame.pack(fill=tk.X, pady=5)

        self.data_prompt_entry = ttk.Entry(prompt_frame, textvariable=self.language_instruction, width=25)
        self.data_prompt_entry.pack(fill=tk.X, pady=2)

        # 프리셋 버튼
        preset_frame = ttk.Frame(prompt_frame)
        preset_frame.pack(fill=tk.X, pady=2)

        presets = ["pick up the object", "place on table", "fold the towel", "open drawer"]
        for i, preset in enumerate(presets):
            ttk.Button(
                preset_frame, text=preset[:12] + ".." if len(preset) > 12 else preset,
                width=12, command=lambda p=preset: self.language_instruction.set(p)
            ).grid(row=i//2, column=i%2, padx=1, pady=1)

        # 통계
        stat_frame = ttk.LabelFrame(parent, text="수집 상태", padding=5)
        stat_frame.pack(fill=tk.X, pady=5)

        stat_inner = ttk.Frame(stat_frame)
        stat_inner.pack(fill=tk.X)

        ttk.Label(stat_inner, text="총 에피소드:").pack(side=tk.LEFT)
        self.data_total_episodes_label = ttk.Label(stat_inner, text="0", font=('Noto Sans CJK KR', 10, 'bold'))
        self.data_total_episodes_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(stat_inner, text="총 프레임:").pack(side=tk.LEFT, padx=(10, 0))
        self.data_total_frames_label = ttk.Label(stat_inner, text="0", font=('Noto Sans CJK KR', 10, 'bold'))
        self.data_total_frames_label.pack(side=tk.LEFT, padx=5)

        # 마지막 에피소드 정보
        self.data_last_episode_info = ttk.Label(stat_frame, text="-", foreground='gray', wraplength=200)
        self.data_last_episode_info.pack(anchor=tk.W, pady=5)

        # 변환 버튼
        conv_frame = ttk.LabelFrame(parent, text="LeRobot 변환", padding=5)
        conv_frame.pack(fill=tk.X, pady=5)

        self.data_convert_btn = tk.Button(
            conv_frame, text="LeRobot 포맷 변환", font=('Noto Sans CJK KR', 10, 'bold'),
            bg='#007bff', fg='white', command=self.convert_to_lerobot_format
        )
        self.data_convert_btn.pack(fill=tk.X, pady=2)

        self.data_conversion_status = ttk.Label(conv_frame, text="준비됨", foreground='gray')
        self.data_conversion_status.pack(anchor=tk.W)

        self.data_conversion_progress = ttk.Progressbar(conv_frame, mode='determinate')
        self.data_conversion_progress.pack(fill=tk.X, pady=2)

        # 폴더 열기
        ttk.Button(
            conv_frame, text="데이터 폴더 열기", command=self.open_data_folder
        ).pack(fill=tk.X, pady=2)

        # 초기 통계 업데이트
        self.update_data_total_stats()

    # ========== 카메라 함수 ==========

    def _load_camera_config(self):
        """저장된 카메라 설정 로드"""
        config_file = Path(__file__).parent / "camera_config.json"
        if config_file.exists():
            try:
                with open(config_file, 'r') as f:
                    return json.load(f)
            except:
                pass
        return {}

    def _save_camera_config(self, config):
        """카메라 설정 저장"""
        config_file = Path(__file__).parent / "camera_config.json"
        try:
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            print(f"카메라 설정 저장 실패: {e}")

    def _detect_cameras(self):
        """연결된 카메라 자동 감지 (USB 포트 경로 기반)"""
        cameras = []
        import os
        import subprocess

        def safe_log(msg):
            """안전하게 로그 출력"""
            try:
                self.log_real(msg)
            except:
                print(msg)

        # 저장된 설정 로드
        saved_config = self._load_camera_config()

        # 내장 카메라 제외 키워드
        internal_keywords = ['FHD Camera', 'IR Camera', 'Integrated', 'Built-in']

        # 외부 카메라 허용 키워드
        external_keywords = ['C270', 'Logitech', 'Orbbec', 'Gemini', 'Astra', 'Dabai', 'USB Camera']

        def get_usb_port_path(video_idx):
            """video 장치의 USB 포트 경로 가져오기 (물리적 포트 식별)"""
            try:
                # /sys/class/video4linux/videoX -> 실제 디바이스 경로
                real_path = os.path.realpath(f"/sys/class/video4linux/video{video_idx}")
                # 경로 예: .../1-2.3.2/1-2.3.2:1.0/video4linux/video0
                parts = real_path.split('/')
                # USB 포트 형식 찾기: 숫자로 시작, - 포함, : 미포함
                usb_ports = []
                for part in parts:
                    if part and part[0].isdigit() and '-' in part and ':' not in part:
                        usb_ports.append(part)
                # 가장 긴 것 반환 (1-2.3.2 > 1-2.3 > 1-2)
                if usb_ports:
                    return max(usb_ports, key=len)
            except:
                pass
            return ""

        safe_log("[카메라] 자동 감지 시작...")

        # 모든 video 장치 스캔
        found_cameras = []
        for i in range(30):  # video0 ~ video29 스캔
            name_path = f"/sys/class/video4linux/video{i}/name"
            if os.path.exists(name_path):
                try:
                    with open(name_path, 'r') as f:
                        name = f.read().strip()

                    # 내장 카메라 제외
                    is_internal = any(kw in name for kw in internal_keywords)
                    if is_internal:
                        continue

                    # 외부 카메라만 허용
                    is_external = any(kw in name for kw in external_keywords)
                    if not is_external:
                        continue

                    # USB 포트 경로
                    usb_port = get_usb_port_path(i)

                    # 카메라 종류 판단
                    fourcc = 'MJPG'
                    if 'C270' in name or 'Logitech' in name:
                        cam_type = 'C270'
                    elif 'Orbbec' in name or 'Gemini' in name or 'Astra' in name or 'Dabai' in name:
                        cam_type = 'Gemini'
                    elif 'USB Camera' in name:
                        cam_type = '기타'
                    else:
                        cam_type = 'Unknown'

                    found_cameras.append({
                        'device': f'/dev/video{i}',
                        'video_idx': i,
                        'name': cam_type,
                        'full_name': name,
                        'fourcc': fourcc,
                        'usb_port': usb_port,
                        'cam_type': cam_type
                    })
                    safe_log(f"  발견: video{i} = {name} (USB: {usb_port})")
                except:
                    pass

        # USB 포트별로 그룹화 (같은 물리 카메라의 여러 video 장치 통합)
        cameras_by_port = {}
        for cam in found_cameras:
            port = cam['usb_port']
            if port not in cameras_by_port:
                cameras_by_port[port] = []
            cameras_by_port[port].append(cam)

        # 각 물리 카메라에서 대표 video 선택 (가장 낮은 video 번호)
        unique_cameras = []
        for port, cams in cameras_by_port.items():
            cams.sort(key=lambda c: c['video_idx'])
            # Gemini는 모든 스트림 유지 (RGB 찾기 위해 라운드로빈)
            if cams[0]['cam_type'] == 'Gemini':
                for cam in cams:
                    unique_cameras.append(cam)
                safe_log(f"  USB {port}: Gemini -> video{cams[0]['video_idx']}~{cams[-1]['video_idx']} ({len(cams)}개 모두 유지)")
            else:
                representative = cams[0]
                unique_cameras.append(representative)
                safe_log(f"  USB {port}: {representative['cam_type']} -> video{representative['video_idx']} 선택 ({len(cams)}개 중)")

        # Gemini와 C270 분리
        gemini_cams = [c for c in unique_cameras if c['cam_type'] == 'Gemini']
        c270_cams = [c for c in unique_cameras if c['cam_type'] == 'C270']
        etc_cams = [c for c in unique_cameras if c['cam_type'] == '기타']

        safe_log(f"[카메라] 물리 카메라: Gemini {len(gemini_cams)}개 스트림, C270 {len(c270_cams)}개, 기타 {len(etc_cams)}개")

        # 저장된 USB 포트 -> 역할 매핑 사용
        # saved_config 예: {"port_roles": {"1-2.3.4": "top", "1-2.3.2": "wrist_right", "1-2.3.3": "wrist_left"}}
        if saved_config and 'port_roles' in saved_config:
            safe_log("[카메라] 저장된 USB 포트 설정 사용")
            port_roles = saved_config['port_roles']

            for port, role in port_roles.items():
                # 해당 USB 포트의 카메라들 찾기
                port_cams = [c for c in found_cameras if c['usb_port'] == port]
                if not port_cams:
                    continue

                port_cams.sort(key=lambda c: c['video_idx'])

                if role == 'top' and len(port_cams) > 0:
                    # Gemini RGB: 자동으로 컬러 스트림 찾기
                    best_cam = None
                    best_score = -1
                    for c in port_cams:
                        vid = c['video_idx']
                        try:
                            cap = cv2.VideoCapture(vid)
                            if cap.isOpened():
                                ret, frame = cap.read()
                                cap.release()
                                if ret and frame is not None:
                                    brightness = np.mean(frame)
                                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                                        b, g, r = cv2.split(frame)
                                        color_diff = np.mean(np.abs(b.astype(float) - g.astype(float))) + \
                                                   np.mean(np.abs(g.astype(float) - r.astype(float)))
                                    else:
                                        color_diff = 0
                                    score = brightness + color_diff
                                    if score > best_score:
                                        best_score = score
                                        best_cam = c
                        except:
                            pass
                    if best_cam and best_score > 50:
                        best_cam['role'] = 'top'
                        cameras.append(best_cam)
                        safe_log(f"  top: video{best_cam['video_idx']} (USB: {port}, 점수={best_score:.0f})")
                else:
                    # C270: 첫번째 video 사용
                    cam = port_cams[0]
                    cam['role'] = role
                    cameras.append(cam)
                    safe_log(f"  {role}: video{cam['video_idx']} (USB: {port})")

            # 역할 순서대로 정렬
            role_order = {'top': 0, 'wrist_right': 1, 'wrist_left': 2}
            cameras.sort(key=lambda c: role_order.get(c.get('role', ''), 99))

            if len(cameras) >= 3:
                for i, cam in enumerate(cameras):
                    safe_log(f"[카메라] {i+1}번: {cam['name']} (video{cam['video_idx']}) - {cam['role']}")
                return cameras
            else:
                safe_log("[카메라] 저장된 설정 불완전, 자동 감지로 전환")
                cameras = []

        # 자동 감지 모드
        safe_log("[카메라] 자동 감지 모드")

        # Gemini RGB 선택 - 라운드로빈 (재검색마다 다른 스트림)
        gemini_rgb = None
        if gemini_cams:
            if not hasattr(self, '_gemini_round_robin_idx'):
                self._gemini_round_robin_idx = 0
            idx = self._gemini_round_robin_idx % len(gemini_cams)
            self._gemini_round_robin_idx += 1
            gemini_rgb = gemini_cams[idx]
            safe_log(f"[카메라] Gemini 탑 선택: video{gemini_rgb['video_idx']} ({idx+1}/{len(gemini_cams)})")

        # C270 정렬 (USB 포트 경로 기준)
        c270_cams.sort(key=lambda c: c['usb_port'])

        # 최종 카메라 목록 구성: [탑뷰, 우손목, 좌손목]
        if gemini_rgb:
            gemini_rgb['role'] = 'top'
            cameras.append(gemini_rgb)

        for idx, cam in enumerate(c270_cams[:2]):
            if idx == 0:
                cam['role'] = 'wrist_right'
            else:
                cam['role'] = 'wrist_left'
            cameras.append(cam)

        # 기타 카메라 추가
        for idx, cam in enumerate(etc_cams):
            cam['role'] = f'기타{idx+1}'
            cameras.append(cam)

        # USB 포트 기반 설정 저장
        new_config = {
            'port_roles': {}
        }
        for cam in cameras:
            new_config['port_roles'][cam['usb_port']] = cam.get('role', 'unknown')
        self._save_camera_config(new_config)
        safe_log("[카메라] USB 포트 설정 저장됨")

        # 로그 출력
        for i, cam in enumerate(cameras):
            role = cam.get('role', 'unknown')
            safe_log(f"[카메라] {i+1}번: {cam['name']} (video{cam['video_idx']}) - {role}")

        return cameras

    def _create_camera_ui(self):
        """감지된 카메라 수에 맞게 UI 생성"""
        # 기존 위젯 제거
        for widget in self.cam_grid.winfo_children():
            widget.destroy()
        self.camera_canvases.clear()
        self.camera_labels.clear()

        num_cams = len(self.detected_cameras)
        if num_cams == 0:
            label = ttk.Label(self.cam_grid, text="연결된 카메라 없음", font=('Noto Sans CJK KR', 12))
            label.pack(pady=20)
            return

        # 그리드 레이아웃 계산 (2열)
        cols = 2
        for i, cam in enumerate(self.detected_cameras):
            row = i // cols
            col = i % cols

            cam_container = ttk.Frame(self.cam_grid)
            cam_container.grid(row=row, column=col, padx=3, pady=2)

            canvas = tk.Canvas(cam_container, width=145, height=110, bg='#2d2d44')
            canvas.pack()
            self.camera_canvases[i+1] = canvas

            # 역할에 따른 라벨 표시
            role = cam.get('role', 'unknown')
            role_names = {
                'top': '탑뷰 (Gemini)',
                'wrist_right': '우손목 (C270)',
                'wrist_left': '좌손목 (C270)'
            }
            label_text = role_names.get(role, f"video{cam['video_idx']}")
            label = ttk.Label(cam_container, text=label_text, font=('Noto Sans CJK KR', 10, 'bold'))
            label.pack()
            self.camera_labels[i+1] = label

        self.log_real(f"[카메라] {num_cams}대 감지됨")
        for i, cam in enumerate(self.detected_cameras):
            self.log_real(f"  {i+1}. {cam['name']} ({cam['device']})")

    def _refresh_cameras(self):
        """카메라 재검색"""
        self.stop_data_camera()
        self.detected_cameras = self._detect_cameras()
        self._create_camera_ui()

    def start_data_camera(self):
        """감지된 카메라 시작 (비동기)"""
        self.stop_camera = False
        self.data_cameras.clear()
        self.camera_frames.clear()

        if not self.detected_cameras:
            self.log_real("[카메라] 연결된 카메라 없음")
            return

        self.log_real(f"[카메라] {len(self.detected_cameras)}대 시작 중...")

        # 각 카메라를 별도 스레드에서 독립 캡처
        for i, cam in enumerate(self.detected_cameras):
            cam_num = i + 1
            self.data_cameras[cam_num] = {
                'type': 'parallel',
                'device': cam['device'],
                'name': cam['name']
            }
            self.log_real(f"[카메라] {cam_num}. {cam['name']} 등록 ({cam['device']})")

        self.log_real(f"[카메라] 병렬 캡처 모드 ({len(self.data_cameras)}대)")
        self._finish_camera_init()

    def _finish_camera_init(self):
        """카메라 초기화 완료 후 호출"""
        if not self.data_cameras:
            self.log_real("[카메라] 연결된 카메라 없음")
            return

        self.log_real(f"[카메라] 총 {len(self.data_cameras)}대 연결됨")

        # 대역폭 계산 및 표시
        cam_count = len(self.data_cameras)
        # 현재 설정: 320x240 15fps MJPG
        width, height, fps = 320, 240, 15
        mjpg_ratio = 0.1  # MJPG 압축률 약 10%
        raw_bw = width * height * 2 * fps  # YUYV 기준 바이트/초
        mjpg_bw = raw_bw * mjpg_ratio
        total_mbps = (mjpg_bw * cam_count * 8) / 1024 / 1024
        usb2_percent = total_mbps / 480 * 100

        self.log_real(f"[대역폭] 설정: {width}x{height} {fps}fps MJPG")
        self.log_real(f"[대역폭] 카메라당: ~{mjpg_bw/1024/1024*8:.1f} Mbps")
        self.log_real(f"[대역폭] 총 {cam_count}대: ~{total_mbps:.1f} Mbps")
        self.log_real(f"[대역폭] USB 2.0 (480Mbps) 사용률: {usb2_percent:.1f}%")

        # 디스플레이 루프 시작
        self._camera_display_active = True
        self._update_camera_display()

        self.data_cam_start_btn.configure(state='disabled')

    def _update_camera_display(self):
        """카메라 디스플레이 시작 - 각 카메라별 독립 스레드"""
        if not self._camera_display_active or self.stop_camera:
            return

        # 각 카메라마다 독립 캡처 스레드 시작
        self._seq_capture_running = True
        for cam_num, cam_info in self.data_cameras.items():
            t = threading.Thread(
                target=self._parallel_capture_worker,
                args=(cam_num, cam_info['device']),
                daemon=True
            )
            t.start()
            self.log_real(f"[카메라] cam{cam_num} 캡처 스레드 시작")

        # GUI 업데이트 루프 시작
        self._gui_display_loop()

    def _parallel_capture_worker(self, cam_num, device):
        """각 카메라 독립 스레드: 열기→연속 읽기 + 자동 재연결"""
        while self._seq_capture_running and not self.stop_camera:
            cap = None
            try:
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    self.root.after(0, lambda cn=cam_num: self.log_real(f"[카메라] cam{cn} 열기 실패, 2초 후 재시도"))
                    time.sleep(2)
                    continue

                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                fail_count = 0
                self.root.after(0, lambda cn=cam_num: self.log_real(f"[카메라] cam{cn} 연결됨"))
                while self._seq_capture_running and not self.stop_camera:
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.camera_frames[cam_num] = frame
                        self.camera_last_change_time[cam_num] = time.time()
                        fail_count = 0
                    else:
                        fail_count += 1
                        if fail_count > 30:
                            self.root.after(0, lambda cn=cam_num: self.log_real(f"[카메라] cam{cn} 끊김 감지, 재연결 시도..."))
                            break
                    time.sleep(0.03)

                cap.release()
                cap = None
                time.sleep(1)

            except Exception:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                time.sleep(2)

    def _gui_display_loop(self):
        """메인 스레드: 캡처된 프레임을 캔버스에 표시"""
        from PIL import Image, ImageTk

        if not self._camera_display_active or self.stop_camera:
            return

        for cam_num in self.data_cameras:
            if cam_num in self.camera_frames and cam_num in self.camera_canvases:
                try:
                    frame = self.camera_frames[cam_num]
                    canvas = self.camera_canvases[cam_num]
                    canvas_w = canvas.winfo_width() or 145
                    canvas_h = canvas.winfo_height() or 110

                    display_frame = cv2.resize(frame, (canvas_w, canvas_h))
                    display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(display_frame)
                    photo = ImageTk.PhotoImage(img)

                    canvas.delete("all")
                    canvas.create_image(0, 0, anchor=tk.NW, image=photo)
                    canvas.image = photo
                except:
                    pass

        if self._camera_display_active:
            self.root.after(66, self._gui_display_loop)

    def _check_camera_freeze(self):
        """카메라 멈춤 감지 및 경고 표시"""
        current_time = time.time()
        frozen_cams = []

        for cam_num in self.camera_last_change_time:
            elapsed = current_time - self.camera_last_change_time[cam_num]
            if elapsed > self.camera_freeze_threshold:
                frozen_cams.append(cam_num)

        if frozen_cams:
            # 경고 표시
            if self.camera_freeze_warning is None:
                # 경고 라벨 생성 (화면 중앙에 크게)
                self.camera_freeze_warning = tk.Label(
                    self.root,
                    text="⚠ 카메라 멈춤! ⚠",
                    font=('Arial', 24, 'bold'),
                    fg='white',
                    bg='red',
                    padx=20,
                    pady=10
                )
                self.camera_freeze_warning.place(relx=0.5, rely=0.1, anchor='center')
            # 멈춘 카메라 번호 표시
            cam_names = [f"CAM{c}" for c in frozen_cams]
            self.camera_freeze_warning.configure(text=f"⚠ 카메라 멈춤: {', '.join(cam_names)} ⚠")
        else:
            # 경고 제거
            if self.camera_freeze_warning is not None:
                self.camera_freeze_warning.destroy()
                self.camera_freeze_warning = None

    def _data_camera_loop(self):
        """사용 안함 - 호환성 유지용"""
        pass

    def _update_camera_canvas(self, cam_num, frame):
        """사용 안함 - 호환성 유지용"""
        pass

    def stop_data_camera(self):
        """카메라 중지"""
        self.stop_camera = True
        self._camera_display_active = False
        self._seq_capture_running = False

        # 카메라 멈춤 경고 제거
        if self.camera_freeze_warning is not None:
            self.camera_freeze_warning.destroy()
            self.camera_freeze_warning = None
        self.camera_last_hash.clear()
        self.camera_last_change_time.clear()

        for cam_num, cam_info in self.data_cameras.items():
            try:
                cam_type = cam_info.get('type', 'opencv')
                if cam_type == 'orbbec':
                    # Orbbec 파이프라인 중지
                    pipeline = cam_info.get('pipeline')
                    if pipeline:
                        pipeline.stop()
                else:
                    # OpenCV 캡처 해제
                    cap = cam_info.get('cap')
                    if cap:
                        cap.release()
            except:
                pass

        # 라벨 초기화
        label_names = {1: "1. Gemini2 탑뷰", 2: "2. C270 우손목", 3: "3. C270 좌손목"}
        for cam_num, name in label_names.items():
            if cam_num in self.camera_labels:
                self.camera_labels[cam_num].config(text=name)

        self.orbbec_pipeline = None
        self.data_cameras.clear()
        self.camera_frames.clear()
        self.data_cam_start_btn.configure(state='normal')
        self.log_real("[카메라] 중지됨")

    # ========== 녹화 함수 ==========

    def toggle_data_recording(self):
        """녹화 토글"""
        if self.is_recording:
            self.stop_data_recording()
        else:
            self.start_data_recording()

    def cancel_data_recording(self):
        """녹화 취소 - 저장하지 않고 취소"""
        if not self.is_recording:
            return

        self.is_recording = False
        self.data_record_btn.configure(text="[REC] Start", bg='#28a745')
        self.data_cancel_btn.configure(state='disabled')

        # 기본자세 잠금 및 로봇 기본자세로 이동
        self.teleop_baseline_locked = True
        self._move_robot_to_baseline()
        self.log_real("[연동] 기본자세 잠금 - 로봇 기본자세로 복귀")

        # 이미 저장된 이미지 파일들 삭제
        episode_dir = getattr(self, 'current_episode_dir', None)
        if episode_dir and episode_dir.exists():
            import shutil
            try:
                shutil.rmtree(episode_dir)
                self.log_real(f"[데이터] 녹화 취소됨 - 에피소드 폴더 삭제: {episode_dir.name}")
            except Exception as e:
                self.log_real(f"[데이터] 폴더 삭제 실패: {e}")

        # 현재 에피소드 데이터 초기화
        self.current_episode = []
        self.frame_count = 0
        self.data_frame_label.configure(text="0")
        self.data_duration_label.configure(text="0.0초")

        self.log_real("[데이터] 녹화 취소됨 (저장 안함)")

    def toggle_teleop_play(self):
        """텔레옵 Play - 비디오/저장 없이 관절 연동만 (텔레오퍼레이션)"""
        if hasattr(self, '_teleop_play_active') and self._teleop_play_active:
            self._stop_teleop_play()
            return

        if not self.real_connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다.")
            return

        if not self.teleop_connected:
            messagebox.showwarning("경고", "텔레옵이 연결되지 않았습니다.")
            return

        if self.is_recording:
            messagebox.showwarning("경고", "녹화 중에는 사용할 수 없습니다.")
            return

        # 텔레옵 연동 시작 (카운트다운 없이 즉시)
        self._teleop_play_active = True
        self.teleop_baseline_locked = False
        self.teleop_play_btn.configure(text="■ Teleop Stop", bg='#dc3545')
        self.log_real("[텔레옵 Play] 시작 - 비디오/저장 없이 관절 연동")

        # 연동이 아직 안 되어 있으면 시작
        if not self.teleop_to_robot_active:
            self.start_teleop_sync_actual()

    def _stop_teleop_play(self):
        """텔레옵 Play 중지"""
        self._teleop_play_active = False
        self.teleop_baseline_locked = True
        self.teleop_play_btn.configure(text="▶ Teleop Play", bg='#ff8c00')
        self.log_real("[텔레옵 Play] 중지")

        # 연동 중지
        if self.teleop_to_robot_active:
            self.stop_teleop_sync()

        # 로봇 기본자세로 복귀
        self._move_robot_to_baseline()

    def play_previous_recording(self):
        """이전 녹화 재생하기"""
        if not self.real_connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다.")
            return

        if self.is_recording:
            messagebox.showwarning("경고", "녹화 중에는 재생할 수 없습니다.")
            return

        # 데이터셋 경로
        from pathlib import Path
        base_path = Path(os.path.join(_GUI_ROOT, "datasets", "rx1_teleop_v1"))

        # 날짜 폴더 목록
        date_folders = sorted([d for d in base_path.iterdir() if d.is_dir()], reverse=True)
        if not date_folders:
            messagebox.showwarning("경고", "저장된 녹화가 없습니다.")
            return

        # 에피소드 선택 다이얼로그
        select_window = tk.Toplevel(self.root)
        select_window.title("녹화 재생 - 에피소드 선택")
        select_window.geometry("500x400")
        select_window.transient(self.root)
        select_window.grab_set()

        # 날짜 선택
        ttk.Label(select_window, text="날짜 선택:", font=('Noto Sans CJK KR', 11, 'bold')).pack(pady=5)
        date_var = tk.StringVar()
        date_combo = ttk.Combobox(select_window, textvariable=date_var, width=30)
        date_combo['values'] = [d.name for d in date_folders]
        date_combo.current(0)
        date_combo.pack(pady=5)

        # 태스크 선택
        ttk.Label(select_window, text="태스크:", font=('Noto Sans CJK KR', 11, 'bold')).pack(pady=5)
        task_var = tk.StringVar()
        task_combo = ttk.Combobox(select_window, textvariable=task_var, width=30)
        task_combo.pack(pady=5)

        # 에피소드 리스트
        ttk.Label(select_window, text="에피소드:", font=('Noto Sans CJK KR', 11, 'bold')).pack(pady=5)
        episode_listbox = tk.Listbox(select_window, width=50, height=10)
        episode_listbox.pack(pady=5, fill=tk.BOTH, expand=True, padx=10)

        def update_tasks(*args):
            date_folder = base_path / date_var.get()
            if date_folder.exists():
                tasks = [d.name for d in date_folder.iterdir() if d.is_dir()]
                task_combo['values'] = tasks
                if tasks:
                    task_combo.current(0)
                    update_episodes()

        def update_episodes(*args):
            episode_listbox.delete(0, tk.END)
            date_folder = base_path / date_var.get()
            task_folder = date_folder / task_var.get()
            if task_folder.exists():
                episodes = sorted([d.name for d in task_folder.iterdir() if d.is_dir() and d.name.startswith('episode_')])
                for ep in episodes:
                    # 프레임 수 확인
                    ep_data_path = task_folder / ep / 'episode_data.json'
                    if ep_data_path.exists():
                        try:
                            with open(ep_data_path) as f:
                                data = json.load(f)
                            episode_listbox.insert(tk.END, f"{ep} ({len(data)} frames)")
                        except:
                            episode_listbox.insert(tk.END, ep)

        date_combo.bind('<<ComboboxSelected>>', update_tasks)
        task_combo.bind('<<ComboboxSelected>>', update_episodes)
        update_tasks()

        def start_playback():
            selection = episode_listbox.curselection()
            if not selection:
                messagebox.showwarning("경고", "에피소드를 선택하세요.")
                return

            ep_text = episode_listbox.get(selection[0])
            ep_name = ep_text.split(' ')[0]  # "episode_0000 (10 frames)" -> "episode_0000"

            episode_path = base_path / date_var.get() / task_var.get() / ep_name / 'episode_data.json'

            select_window.destroy()
            self._playback_episode(str(episode_path))

        # 버튼
        btn_frame = ttk.Frame(select_window)
        btn_frame.pack(pady=10)

        tk.Button(btn_frame, text="▶ 재생", font=('Noto Sans CJK KR', 12, 'bold'),
                  bg='#28a745', fg='white', width=10, command=start_playback).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="취소", font=('Noto Sans CJK KR', 12),
                  bg='#6c757d', fg='white', width=10, command=select_window.destroy).pack(side=tk.LEFT, padx=5)

    def _playback_episode(self, episode_path):
        """에피소드 재생 실행"""
        try:
            with open(episode_path, 'r') as f:
                frames = json.load(f)

            if not frames:
                messagebox.showwarning("경고", "빈 에피소드입니다.")
                return

            total_frames = len(frames)
            self.log_real(f"[재생] 시작: {episode_path}")
            self.log_real(f"[재생] 총 프레임: {total_frames}")

            # ACC 설정 (GUI 기본값과 동일하게 6)
            acc_value = self.real_acceleration_value.get()
            for joint_id in self.learning_joint_ids:
                self.set_real_servo_acceleration(joint_id, acc_value)

            # 재생 버튼 비활성화
            self.data_playback_btn.configure(state='disabled', text="Playing...")

            # 별도 스레드에서 재생
            def playback_thread():
                try:
                    fps = self.recording_fps
                    interval = 1.0 / fps

                    for i, frame in enumerate(frames):
                        if not self.real_connected:
                            break

                        state_raw = frame.get('observation.state_raw', [])
                        if len(state_raw) == len(self.learning_joint_ids):
                            positions = {}
                            for j, joint_id in enumerate(self.learning_joint_ids):
                                positions[joint_id] = state_raw[j]
                            self.sync_write_positions(positions)

                        # UI 업데이트
                        self.root.after(0, lambda f=i+1, t=total_frames:
                            self.log_real(f"[재생] 프레임 {f}/{t}") if f % 10 == 0 else None)

                        time.sleep(interval)

                    self.root.after(0, lambda: self.log_real("[재생] 완료!"))
                    self.root.after(0, lambda: messagebox.showinfo("재생 완료", "에피소드 재생이 완료되었습니다."))

                except Exception as e:
                    self.root.after(0, lambda: self.log_real(f"[재생] 오류: {e}"))
                finally:
                    self.root.after(0, lambda: self.data_playback_btn.configure(
                        state='normal', text="▶ Play Recording"))

            threading.Thread(target=playback_thread, daemon=True).start()

        except Exception as e:
            self.log_real(f"[재생] 로드 실패: {e}")
            messagebox.showerror("오류", f"에피소드 로드 실패: {e}")

    def _move_robot_to_baseline(self):
        """로봇을 기본자세로 빠르게 이동 (녹화 종료 시 사용)"""
        if not self.real_connected:
            return
        try:
            # Sync Write로 한 번에 기본자세 전송
            if hasattr(self, 'robot_baseline') and self.robot_baseline:
                self.sync_write_positions(self.robot_baseline)
        except Exception as e:
            self.log_real(f"[연동] 기본자세 이동 실패: {e}")

    def _toggle_keyboard_recording(self):
        """키보드 녹화 컨트롤 토글"""
        enabled = self.keyboard_rec_var.get()
        self.keyboard_recording_enabled = enabled

        if enabled:
            # 키보드 이벤트 바인딩 (bind_all로 전역 캡처)
            self.root.bind_all('<space>', self._on_space_key)
            self.root.bind_all('<Left>', self._on_arrow_key)
            self.root.bind_all('<Right>', self._on_arrow_key)
            self.root.bind_all('<Up>', self._on_arrow_key)
            self.root.bind_all('<Down>', self._on_arrow_key)
            self.log_real("[녹화] 키보드 컨트롤 활성화 (Space: 시작/저장, 방향키: 취소)")
        else:
            # 키보드 이벤트 해제
            self.root.unbind_all('<space>')
            self.root.unbind_all('<Left>')
            self.root.unbind_all('<Right>')
            self.root.unbind_all('<Up>')
            self.root.unbind_all('<Down>')
            self.log_real("[녹화] 키보드 컨트롤 비활성화")

    def _on_space_key(self, event):
        """스페이스바 - 녹화 시작/저장"""
        if not self.keyboard_recording_enabled:
            return
        # Entry/Text 위젯에서 입력 중일 때는 무시
        if isinstance(event.widget, (tk.Entry, ttk.Entry, tk.Text)):
            return
        self.toggle_data_recording()
        return "break"  # 이벤트 전파 중지 (버튼 눌림 방지)

    def _on_arrow_key(self, event):
        """방향키 - 녹화 취소"""
        if not self.keyboard_recording_enabled:
            return
        # Entry/Text 위젯에서 입력 중일 때는 무시
        if isinstance(event.widget, (tk.Entry, ttk.Entry, tk.Text)):
            return
        if self.is_recording:
            self.cancel_data_recording()
        return "break"  # 이벤트 전파 중지

    def _get_next_episode_number(self):
        """기존 에피소드 폴더를 확인하고 다음 번호 반환 (오늘 날짜 폴더 기준)"""
        try:
            save_dir = self._get_save_dir(use_recording_date=False)  # 오늘 날짜 사용

            if not save_dir.exists():
                return 0

            # episode_XXXX 폴더들 찾기
            existing = []
            for folder in save_dir.glob("episode_*"):
                if folder.is_dir():
                    try:
                        num = int(folder.name.split("_")[1])
                        existing.append(num)
                    except:
                        pass

            if existing:
                next_num = max(existing) + 1
                print(f"[데이터] 기존 에피소드 {len(existing)}개 발견, 다음 번호: {next_num}")
                return next_num
        except Exception as e:
            print(f"[데이터] 에피소드 번호 확인 오류: {e}")
        return 0

    def _get_save_dir(self, use_recording_date=True):
        """저장 경로 반환 (날짜 폴더 포함)

        구조: datasets/rx1_teleop_v1/YYYY_MM_DD/pick_and_place/
        """
        from datetime import datetime

        # 날짜 결정: 녹화 중이면 녹화 시작 날짜, 아니면 오늘
        if use_recording_date and self.recording_date:
            date_str = self.recording_date
        else:
            date_str = datetime.now().strftime("%Y_%m_%d")

        save_dir = Path(self.base_data_dir) / self.current_dataset_name.get() / date_str / self.current_task_name.get()
        return save_dir

    def start_data_recording(self):
        """녹화 시작"""
        if not self.real_connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다.")
            return

        if not self.data_cameras:
            messagebox.showwarning("경고", "카메라를 먼저 시작하세요.")
            return

        self.is_recording = True
        self.current_episode = []
        self.frame_count = 0
        self.recording_start_time = time.time()

        # 녹화 시작 날짜 기록 (자정 넘어가도 같은 폴더에 저장)
        from datetime import datetime
        self.recording_date = datetime.now().strftime("%Y_%m_%d")

        # 스트리밍 저장용: 에피소드 폴더 미리 생성 (날짜 폴더 포함)
        save_dir = self._get_save_dir()
        save_dir.mkdir(parents=True, exist_ok=True)
        self.current_episode_dir = save_dir / f"episode_{self.episode_count:04d}"
        self.current_episode_dir.mkdir(exist_ok=True)

        # 이미지 폴더 미리 생성
        self.streaming_image_dirs = {}
        for cam_num in self.data_cameras.keys():
            cam_key = f'cam_{cam_num}'
            pi0_key = self.pi0_camera_keys.get(cam_key, f'observation.images.cam_{cam_num}')
            folder_name = pi0_key.replace('.', '_')
            cam_dir = self.current_episode_dir / folder_name
            cam_dir.mkdir(exist_ok=True)
            self.streaming_image_dirs[cam_num] = {'dir': cam_dir, 'pi0_key': pi0_key}

        self.data_record_btn.configure(text="[STOP] Save", bg='#dc3545')
        self.data_cancel_btn.configure(state='normal')
        self.log_real(f"[데이터] 녹화 시작 - 에피소드 {self.episode_count + 1} (스트리밍 저장)")

        # 0.5초 후 기본자세 잠금 해제 (텔레옵 연동 활성화)
        def unlock_baseline():
            if self.is_recording:  # 아직 녹화 중이면
                self.teleop_baseline_locked = False
                self.log_real("[연동] 기본자세 잠금 해제 - 텔레옵 연동 시작")
        self.root.after(500, unlock_baseline)

        # 녹화 스레드 시작
        self.recording_thread = threading.Thread(target=self._data_recording_loop, daemon=True)
        self.recording_thread.start()

    def _data_recording_loop(self):
        """녹화 루프 - Pi0.5 LeRobot v2 포맷 (스트리밍 저장)"""
        interval = 1.0 / self.recording_fps
        import gc

        while self.is_recording:
            start_time = time.time()

            # Pi0.5 LeRobot v2 포맷 프레임 데이터 (이미지 제외, JSON만)
            frame_data = {
                'timestamp': time.time() - self.recording_start_time,
                'frame_index': self.frame_count,
                'episode_index': self.episode_count,
            }

            # 카메라 프레임 -> 바로 디스크에 저장 (RAM 절약)
            for cam_num, frame in self.camera_frames.items():
                if frame is not None and cam_num in self.streaming_image_dirs:
                    cam_info = self.streaming_image_dirs[cam_num]
                    img_path = cam_info['dir'] / f"frame_{self.frame_count:06d}.jpg"
                    cv2.imwrite(str(img_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

            # 관절 상태 (observation.state) - 실제 로봇 서보 피드백 사용
            # ★ 수정 (2026-01-28): 텔레옵 루프에서 캐시한 실제 위치 사용 (시리얼 충돌 방지)
            state_raw = []  # raw 값 (0-4095)
            state_normalized = []  # 정규화 값 (-1 to 1)

            # 텔레옵 루프에서 캐시한 실제 로봇 위치 사용
            servo_positions = self.cached_robot_positions

            for joint_id in self.learning_joint_ids:
                if joint_id in servo_positions:
                    raw_pos = servo_positions[joint_id]
                else:
                    # 캐시에 없으면 명령값으로 폴백
                    if joint_id in self.real_active_joints:
                        raw_pos = self.real_active_joints[joint_id]['current']
                    else:
                        raw_pos = self.servo_center  # 기본값 2048

                state_raw.append(raw_pos)
                # 정규화: (pos - 2048) / 2048 -> -1 to 1
                normalized = (raw_pos - self.servo_center) / self.servo_center
                state_normalized.append(normalized)

            frame_data['observation.state'] = state_normalized
            frame_data['observation.state_raw'] = state_raw  # 원본 값도 저장

            # 언어 명령 (task)
            frame_data['task'] = self.language_instruction.get()

            # next.done 플래그 (에피소드 끝 표시용, 나중에 설정)
            frame_data['next.done'] = False

            self.current_episode.append(frame_data)
            self.frame_count += 1

            # 100프레임마다 가비지 컬렉션
            if self.frame_count % 100 == 0:
                gc.collect()

            # UI 업데이트
            self.root.after(0, self._update_data_recording_ui)

            # FPS 유지
            elapsed = time.time() - start_time
            if elapsed < interval:
                time.sleep(interval - elapsed)

    def _update_data_recording_ui(self):
        """녹화 UI 업데이트"""
        self.data_frame_label.configure(text=str(self.frame_count))
        duration = time.time() - self.recording_start_time if self.recording_start_time else 0
        self.data_duration_label.configure(text=f"{duration:.1f}초")

    def stop_data_recording(self):
        """녹화 중지 및 저장"""
        self.is_recording = False
        self.data_record_btn.configure(text="[REC] Start", bg='#28a745')
        self.data_cancel_btn.configure(state='disabled')

        # 기본자세 잠금 및 로봇 기본자세로 이동
        self.teleop_baseline_locked = True
        self._move_robot_to_baseline()
        self.log_real("[연동] 기본자세 잠금 - 로봇 기본자세로 복귀")

        if len(self.current_episode) > 0:
            self.save_data_episode()
            self.episode_count += 1
            self.data_episode_label.configure(text=str(self.episode_count))

            duration = self.current_episode[-1]['timestamp'] if self.current_episode else 0
            self.data_last_episode_info.configure(
                text=f"에피소드 {self.episode_count}: {len(self.current_episode)} 프레임, {duration:.1f}초"
            )
            self.log_real(f"[데이터] 에피소드 {self.episode_count} 저장됨: {len(self.current_episode)} 프레임")

    def save_data_episode(self):
        """에피소드 저장 - Pi0.5 LeRobot v2 포맷 (스트리밍 저장 모드)"""
        # 스트리밍 저장에서는 이미 current_episode_dir이 생성됨
        episode_dir = getattr(self, 'current_episode_dir', None)
        if episode_dir is None:
            # fallback: 기존 방식 (날짜 폴더 포함)
            save_dir = self._get_save_dir()
            save_dir.mkdir(parents=True, exist_ok=True)
            episode_dir = save_dir / f"episode_{self.episode_count:04d}"
            episode_dir.mkdir(exist_ok=True)

        # 마지막 프레임에 next.done = True 설정
        if self.current_episode:
            self.current_episode[-1]['next.done'] = True

        # 액션 계산 (다음 프레임의 state가 현재 프레임의 action)
        for i, frame_data in enumerate(self.current_episode):
            if i < len(self.current_episode) - 1:
                # 다음 프레임의 state가 action
                frame_data['action'] = self.current_episode[i + 1]['observation.state'].copy()
            else:
                # 마지막 프레임은 현재 state 유지
                frame_data['action'] = frame_data['observation.state'].copy()

        # 이미지는 스트리밍 녹화 중 이미 저장됨 (RAM 절약)
        # 별도의 이미지 저장 과정 없음

        # Pi0.5 LeRobot v2 메타데이터
        learning_joint_names = []
        for jid in self.learning_joint_ids:
            if jid in self.real_active_joints:
                learning_joint_names.append(self.real_active_joints[jid]['name'])

        metadata = {
            # LeRobot v2 필수 필드
            'codebase_version': '2.0',
            'robot_type': 'rx1_dual_arm',
            'fps': self.recording_fps,
            'episode_index': self.episode_count,
            'num_frames': len(self.current_episode),
            'duration': self.current_episode[-1]['timestamp'] if self.current_episode else 0,

            # Pi0.5 호환 필드
            'task': self.language_instruction.get(),
            'language_instruction': self.language_instruction.get(),

            # 데이터 차원
            'state_dim': len(self.learning_joint_ids),  # 16
            'action_dim': len(self.learning_joint_ids),  # 16

            # 카메라 정보
            'cameras': list(self.data_cameras.keys()),
            'camera_keys': [self.pi0_camera_keys.get(f'cam_{c}', f'observation.images.{c}')
                           for c in self.data_cameras.keys()],

            # 관절 정보
            'joint_names': learning_joint_names,
            'joint_ids': self.learning_joint_ids,

            # 정규화 정보
            'state_normalization': {
                'type': 'center_scale',
                'center': self.servo_center,
                'scale': self.servo_center,
                'range': [-1.0, 1.0],
            },

            # 타임스탬프
            'created': datetime.now().isoformat(),
        }

        with open(episode_dir / 'metadata.json', 'w') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)

        # Pi0.5 LeRobot v2 데이터 저장 (parquet 호환 JSON)
        episode_data = []
        for i, frame_data in enumerate(self.current_episode):
            row = {
                'episode_index': self.episode_count,
                'frame_index': i,
                'timestamp': frame_data['timestamp'],
                'task': frame_data['task'],

                # observation.state (정규화된 16차원)
                'observation.state': frame_data['observation.state'],

                # action (정규화된 16차원)
                'action': frame_data['action'],

                # next.done
                'next.done': frame_data['next.done'],

                # 원본 값 (디버깅용)
                'observation.state_raw': frame_data['observation.state_raw'],
            }

            # 이미지 경로 추가
            for cam_name in self.data_cameras.keys():
                cam_key = f'cam_{cam_name}'  # 정수를 'cam_1', 'cam_2', 'cam_3' 형식으로 변환
                pi0_key = self.pi0_camera_keys.get(cam_key, f'observation.images.{cam_name}')
                folder_name = pi0_key.replace('.', '_')
                row[pi0_key] = f"{folder_name}/frame_{i:06d}.jpg"

            episode_data.append(row)

        with open(episode_dir / 'episode_data.json', 'w') as f:
            json.dump(episode_data, f, indent=2)

        # 통계 업데이트 (녹화 시작 날짜 기준)
        save_dir = self._get_save_dir()
        self.update_data_total_stats()
        self.update_normalization_stats(save_dir)

    def update_data_total_stats(self):
        """총 통계 업데이트 (현재 날짜 폴더 기준)"""
        save_dir = self._get_save_dir()

        total_episodes = 0
        total_frames = 0

        if save_dir.exists():
            for ep_dir in save_dir.iterdir():
                if ep_dir.is_dir() and ep_dir.name.startswith('episode_'):
                    total_episodes += 1
                    meta_file = ep_dir / 'metadata.json'
                    if meta_file.exists():
                        with open(meta_file) as f:
                            meta = json.load(f)
                            total_frames += meta.get('num_frames', 0)

        self.data_total_episodes_label.configure(text=str(total_episodes))
        self.data_total_frames_label.configure(text=str(total_frames))

    def update_data_fps(self):
        """FPS 업데이트"""
        try:
            self.recording_fps = int(self.data_fps_var.get())
            self.log_real(f"[데이터] FPS 설정: {self.recording_fps}")
        except:
            pass

    def open_data_folder(self):
        """데이터 폴더 열기"""
        path = Path(self.base_data_dir) / self.current_dataset_name.get()
        path.mkdir(parents=True, exist_ok=True)
        os.system(f'xdg-open "{path}"')

    # ========== Pi0.5 정규화 통계 ==========

    def update_normalization_stats(self, save_dir):
        """Pi0.5 학습에 필요한 정규화 통계 업데이트 (MEAN_STD)"""
        try:
            all_states = []
            all_actions = []

            # 모든 에피소드에서 데이터 수집
            for ep_dir in save_dir.iterdir():
                if ep_dir.is_dir() and ep_dir.name.startswith('episode_'):
                    data_file = ep_dir / 'episode_data.json'
                    if data_file.exists():
                        with open(data_file) as f:
                            episode_data = json.load(f)
                        for frame in episode_data:
                            if 'observation.state' in frame:
                                all_states.append(frame['observation.state'])
                            if 'action' in frame:
                                all_actions.append(frame['action'])

            if not all_states:
                return

            # numpy 배열로 변환
            states_np = np.array(all_states, dtype=np.float32)
            actions_np = np.array(all_actions, dtype=np.float32)

            # Pi0.5 정규화 통계 계산 (MEAN_STD 방식)
            stats = {
                'observation.state': {
                    'mean': states_np.mean(axis=0).tolist(),
                    'std': states_np.std(axis=0).tolist(),
                    'min': states_np.min(axis=0).tolist(),
                    'max': states_np.max(axis=0).tolist(),
                },
                'action': {
                    'mean': actions_np.mean(axis=0).tolist(),
                    'std': actions_np.std(axis=0).tolist(),
                    'min': actions_np.min(axis=0).tolist(),
                    'max': actions_np.max(axis=0).tolist(),
                },
                # Pi0.5 호환 형식
                'STATE': {
                    'mean': states_np.mean(axis=0).tolist(),
                    'std': states_np.std(axis=0).tolist(),
                },
                'ACTION': {
                    'mean': actions_np.mean(axis=0).tolist(),
                    'std': actions_np.std(axis=0).tolist(),
                },
            }

            # 통계 파일 저장
            stats_file = save_dir / 'stats.json'
            with open(stats_file, 'w') as f:
                json.dump(stats, f, indent=2)

            self.log_real(f"[데이터] 정규화 통계 업데이트됨: {stats_file}")

        except Exception as e:
            self.log_real(f"[데이터] 정규화 통계 업데이트 실패: {e}")

    # ========== Pi0.5 LeRobot v2 변환 ==========

    def convert_to_lerobot_format(self):
        """Pi0.5 LeRobot v2 포맷으로 변환"""
        if not H5PY_AVAILABLE:
            messagebox.showerror("오류", "h5py가 설치되지 않았습니다.\npip3 install h5py")
            return

        source_dir = self._get_save_dir()  # 현재 날짜 폴더 기준

        if not source_dir.exists():
            messagebox.showerror("오류", f"데이터셋을 찾을 수 없습니다:\n{source_dir}")
            return

        self.data_conversion_status.configure(text="Pi0.5 변환 중...", foreground='blue')
        self.data_convert_btn.configure(state='disabled')

        thread = threading.Thread(target=self._convert_to_pi05_lerobot_thread, args=(source_dir,), daemon=True)
        thread.start()

    def _convert_to_pi05_lerobot_thread(self, source_dir):
        """Pi0.5 LeRobot v2 변환 스레드"""
        try:
            output_dir = source_dir.parent / f"{source_dir.name}_pi05_lerobot"
            output_dir.mkdir(exist_ok=True)

            episodes = sorted([d for d in source_dir.iterdir() if d.is_dir() and d.name.startswith('episode_')])

            all_states = []
            all_actions = []
            all_data = []

            for i, ep_dir in enumerate(episodes):
                progress = (i + 1) / len(episodes) * 100
                self.root.after(0, lambda p=progress: self.data_conversion_progress.configure(value=p))

                # Pi0.5 포맷 데이터 파일 읽기
                data_file = ep_dir / 'episode_data.json'
                meta_file = ep_dir / 'metadata.json'

                if not data_file.exists() or not meta_file.exists():
                    continue

                with open(meta_file) as f:
                    metadata = json.load(f)
                with open(data_file) as f:
                    episode_data = json.load(f)

                for j, frame in enumerate(episode_data):
                    # Pi0.5 키 형식 유지
                    row = {
                        'episode_index': i,
                        'frame_index': j,
                        'index': len(all_data),  # 전체 인덱스
                        'timestamp': frame['timestamp'],
                        'task': frame['task'],
                        'observation.state': frame['observation.state'],
                        'action': frame['action'],
                        'next.done': frame['next.done'],
                    }

                    # 이미지 경로 (절대 경로로 변환)
                    for cam_key in metadata.get('camera_keys', []):
                        if cam_key in frame:
                            row[cam_key] = str(ep_dir / frame[cam_key])

                    all_data.append(row)
                    all_states.append(frame['observation.state'])
                    all_actions.append(frame['action'])

            # HDF5 저장 (Pi0.5 호환)
            self._save_pi05_hdf5(output_dir, all_data, episodes)

            # 정규화 통계 저장
            self._save_pi05_norm_stats(output_dir, all_states, all_actions)

            # 메타데이터 저장
            self._save_pi05_metadata(output_dir, len(episodes), len(all_data))

            self.root.after(0, lambda: self.data_conversion_status.configure(text="Pi0.5 변환 완료!", foreground='green'))
            self.root.after(0, lambda: self.data_convert_btn.configure(state='normal'))
            self.root.after(0, lambda: self.log_real(f"[데이터] {len(episodes)}개 에피소드 Pi0.5 LeRobot 변환 완료"))

        except Exception as e:
            import traceback
            self.root.after(0, lambda: self.data_conversion_status.configure(text=f"오류: {e}", foreground='red'))
            self.root.after(0, lambda: self.data_convert_btn.configure(state='normal'))
            self.root.after(0, lambda: self.log_real(f"[데이터] Pi0.5 변환 오류: {e}\n{traceback.format_exc()}"))

    def _save_pi05_hdf5(self, output_dir, all_data, episodes):
        """Pi0.5 호환 HDF5 형식으로 저장"""
        hdf5_path = output_dir / 'data.hdf5'

        with h5py.File(hdf5_path, 'w') as f:
            # 전체 데이터셋
            num_frames = len(all_data)

            # observation.state (N x 16)
            states = np.array([d['observation.state'] for d in all_data], dtype=np.float32)
            f.create_dataset('observation.state', data=states)

            # action (N x 16)
            actions = np.array([d['action'] for d in all_data], dtype=np.float32)
            f.create_dataset('action', data=actions)

            # 인덱스들
            f.create_dataset('episode_index', data=np.array([d['episode_index'] for d in all_data], dtype=np.int64))
            f.create_dataset('frame_index', data=np.array([d['frame_index'] for d in all_data], dtype=np.int64))
            f.create_dataset('index', data=np.array([d['index'] for d in all_data], dtype=np.int64))
            f.create_dataset('timestamp', data=np.array([d['timestamp'] for d in all_data], dtype=np.float32))
            f.create_dataset('next.done', data=np.array([d['next.done'] for d in all_data], dtype=bool))

            # task (언어 명령) - 문자열 배열
            tasks = [d['task'].encode('utf-8') for d in all_data]
            dt = h5py.special_dtype(vlen=str)
            f.create_dataset('task', data=tasks, dtype=dt)

            # 에피소드별 그룹 (추가 호환성)
            for ep_idx, ep_dir in enumerate(episodes):
                ep_data = [d for d in all_data if d['episode_index'] == ep_idx]
                if not ep_data:
                    continue

                ep_group = f.create_group(f'episode_{ep_idx}')
                ep_group.create_dataset('observation.state',
                    data=np.array([d['observation.state'] for d in ep_data], dtype=np.float32))
                ep_group.create_dataset('action',
                    data=np.array([d['action'] for d in ep_data], dtype=np.float32))
                ep_group.attrs['task'] = ep_data[0]['task']
                ep_group.attrs['num_frames'] = len(ep_data)

    def _save_pi05_norm_stats(self, output_dir, all_states, all_actions):
        """Pi0.5 정규화 통계 저장 (compute_norm_stats.py 호환)"""
        states_np = np.array(all_states, dtype=np.float32)
        actions_np = np.array(all_actions, dtype=np.float32)

        # Pi0.5 OpenPI 호환 형식
        stats = {
            # LeRobot 키
            'observation.state': {
                'mean': states_np.mean(axis=0).tolist(),
                'std': np.maximum(states_np.std(axis=0), 1e-6).tolist(),  # 0 방지
                'min': states_np.min(axis=0).tolist(),
                'max': states_np.max(axis=0).tolist(),
                'q01': np.percentile(states_np, 1, axis=0).tolist(),
                'q99': np.percentile(states_np, 99, axis=0).tolist(),
            },
            'action': {
                'mean': actions_np.mean(axis=0).tolist(),
                'std': np.maximum(actions_np.std(axis=0), 1e-6).tolist(),
                'min': actions_np.min(axis=0).tolist(),
                'max': actions_np.max(axis=0).tolist(),
                'q01': np.percentile(actions_np, 1, axis=0).tolist(),
                'q99': np.percentile(actions_np, 99, axis=0).tolist(),
            },
            # OpenPI 키 (대문자)
            'STATE': {
                'mean': states_np.mean(axis=0).tolist(),
                'std': np.maximum(states_np.std(axis=0), 1e-6).tolist(),
            },
            'ACTION': {
                'mean': actions_np.mean(axis=0).tolist(),
                'std': np.maximum(actions_np.std(axis=0), 1e-6).tolist(),
            },
        }

        with open(output_dir / 'stats.json', 'w') as f:
            json.dump(stats, f, indent=2)

        # norm_stats.json (OpenPI 호환)
        norm_stats = {
            'normalization_mapping': {
                'ACTION': 'MEAN_STD',
                'STATE': 'MEAN_STD',
                'VISUAL': 'IDENTITY',
            },
            'stats': stats,
        }

        with open(output_dir / 'norm_stats.json', 'w') as f:
            json.dump(norm_stats, f, indent=2)

    def _save_pi05_metadata(self, output_dir, num_episodes, num_frames):
        """Pi0.5 LeRobot v2 메타데이터 저장"""
        learning_joint_names = []
        for jid in self.learning_joint_ids:
            if jid in self.real_active_joints:
                learning_joint_names.append(self.real_active_joints[jid]['name'])

        metadata = {
            # LeRobot v2 필수 필드
            'codebase_version': '2.0',
            'robot_type': 'rx1_dual_arm',
            'fps': self.recording_fps,
            'num_episodes': num_episodes,
            'total_frames': num_frames,

            # Pi0.5 호환 필드
            'features': {
                'observation.state': {
                    'dtype': 'float32',
                    'shape': [len(self.learning_joint_ids)],
                    'names': learning_joint_names,
                },
                'action': {
                    'dtype': 'float32',
                    'shape': [len(self.learning_joint_ids)],
                    'names': learning_joint_names,
                },
                'observation.images.cam_high': {
                    'dtype': 'image',
                    'shape': [480, 640, 3],
                    'names': ['height', 'width', 'channel'],
                },
            },

            # 로봇 정보
            'robot': 'rx1',
            'state_dim': len(self.learning_joint_ids),
            'action_dim': len(self.learning_joint_ids),
            'action_horizon': 10,  # Pi0.5 기본값

            # 관절 정보
            'joint_names': learning_joint_names,
            'joint_ids': self.learning_joint_ids,

            # 카메라 정보 (동적으로 연결된 카메라 반영)
            'cameras': [f'cam_{i}' for i in self.data_cameras.keys()],
            'camera_keys': [f'observation.images.cam_{i}' for i in self.data_cameras.keys()],
            'camera_count': len(self.data_cameras),

            # OpenPI 호환
            'openpi_compatible': True,
            'pi05_ready': True,

            # 타임스탬프
            'created': datetime.now().isoformat(),
            'note': 'RX-1 Dual Arm: 16 DOF (7+1 right arm + 7+1 left arm)',
        }

        with open(output_dir / 'metadata.json', 'w') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)

        # info.json (LeRobot 호환)
        info = {
            'codebase_version': '2.0',
            'fps': self.recording_fps,
            'robot_type': 'rx1_dual_arm',
            'total_episodes': num_episodes,
            'total_frames': num_frames,
        }

        with open(output_dir / 'info.json', 'w') as f:
            json.dump(info, f, indent=2)


def main():
    print("RRR GUI Controller v5 시작")
    print("Real 하드웨어 제어 + Isaac Sim 연동 + 데이터 수집 통합")

    controller = RobotController()
    controller.run()



if __name__ == "__main__":
    main()
