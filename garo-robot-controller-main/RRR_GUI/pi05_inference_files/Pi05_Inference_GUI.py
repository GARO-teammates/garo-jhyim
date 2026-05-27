#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi0.5 Inference GUI
학습된 Pi0.5 모델을 사용한 추론 전용 GUI
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import json
import math
import os
import cv2
import numpy as np
from pathlib import Path
from PIL import Image, ImageTk
from datetime import datetime

# PyTorch and LeRobot imports
try:
    import torch
    from lerobot.policies.pi05 import PI05Config, PI05Policy
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    print("Warning: LeRobot not available. Install with: pip install lerobot")

# 음성 인식 imports
try:
    import speech_recognition as sr
    SPEECH_AVAILABLE = True
except ImportError:
    SPEECH_AVAILABLE = False
    print("Warning: SpeechRecognition not available. Install with: pip install SpeechRecognition")

# 번역 imports (한국어 -> 영어)
try:
    from googletrans import Translator
    TRANSLATE_AVAILABLE = True
except ImportError:
    TRANSLATE_AVAILABLE = False
    print("Warning: googletrans not available. Install with: pip install googletrans==4.0.0-rc1")


class Pi05InferenceGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Pi0.5 Inference GUI")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1a1a2e')

        # ========== 경로 설정 ==========
        _this_dir = os.path.dirname(os.path.abspath(__file__))
        _gui_root = os.path.dirname(_this_dir)
        _project_root = os.path.dirname(_gui_root)
        self.models_dir = os.path.join(_project_root, "pi0.5_trained")
        self.baseline_file = os.path.join(_gui_root, "1_Robot_GUI", "rx1_baseline_v5.json")

        # ========== 디버그 로그 파일 ==========
        self.debug_log_dir = os.path.join(_gui_root, "1_Robot_GUI", "debug_logs")
        os.makedirs(self.debug_log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.debug_log_file = os.path.join(self.debug_log_dir, f"debug_{timestamp}.txt")
        self._init_debug_log()

        # ========== 모델 관련 ==========
        self.available_models = []
        self.selected_model = tk.StringVar(value="")
        self.policy = None
        self.model_loaded = False
        self.device = "cuda" if torch.cuda.is_available() else "cpu" if LEROBOT_AVAILABLE else "cpu"

        # ========== 로봇 연결 ==========
        self.serial_port = None
        self.connected = False
        self.baseline_positions = {}

        # ========== 추론 상태 ==========
        self.is_inferencing = False
        self.inference_thread = None
        self.inference_fps = 11  # 11 FPS

        # ========== 카메라 ==========
        self.cameras = {}  # {name: capture}
        self.camera_frames = {}  # {name: frame}
        self.camera_canvases = {}
        self.stop_camera = False
        self.camera_thread = None

        # Pi0.5 카메라 매핑
        self.camera_names = ['top', 'wrist_left', 'wrist_right']

        # ========== 관절 ID ==========
        self.joint_ids = [
            11, 12, 13, 14, 15, 16, 17, 41,  # 오른팔 + 그리퍼
            21, 22, 23, 24, 25, 26, 27, 31,  # 왼팔 + 그리퍼
        ]

        # ========== QUANTILES 정규화 통계 (stats.json에서) ==========
        # 이 값들은 LINEAR 정규화된 공간 ((pos-2048)/2048)에서의 min/max
        self.state_min = np.array([
            -0.6259765625, -0.40869140625, 0.4033203125, -1.0, 0.04931640625, -0.12109375, 0.30859375, -1.0,
            -0.95361328125, -0.40478515625, -1.0, -0.40966796875, 0.048828125, -0.12451171875, -0.30859375, -1.0
        ], dtype=np.float32)
        self.state_max = np.array([
            0.953125, 0.02197265625, 0.99951171875, -0.06201171875, 0.37939453125, 0.171875, 0.30859375, 0.99951171875,
            0.8232421875, 0.58984375, -0.14013671875, 0.99951171875, 0.5693359375, 0.21142578125, 0.15869140625, 0.99951171875
        ], dtype=np.float32)
        self.action_min = np.array([
            -0.6259765625, -0.40869140625, 0.4033203125, -1.0, 0.04931640625, -0.12109375, 0.30859375, -1.0,
            -0.95361328125, -0.40478515625, -1.0, -0.40966796875, 0.048828125, -0.12451171875, -0.30859375, -1.0
        ], dtype=np.float32)
        self.action_max = np.array([
            0.953125, 0.02197265625, 0.99951171875, -0.06201171875, 0.37939453125, 0.171875, 0.30859375, 0.99951171875,
            0.8232421875, 0.58984375, -0.14013671875, 0.99951171875, 0.5693359375, 0.21142578125, 0.15869140625, 0.99951171875
        ], dtype=np.float32)

        # ========== 음성 인식 ==========
        self.speech_recognizer = sr.Recognizer() if SPEECH_AVAILABLE else None
        self.translator = Translator() if TRANSLATE_AVAILABLE else None
        self.current_command = ""  # 현재 언어 명령
        self.language_tokens = None  # 토큰화된 명령
        self.language_attention_mask = None
        self.is_recording = False

        # 초기화
        self.scan_models()
        self.load_baseline()
        self.create_ui()
        self.start_camera_thread()

        # 종료 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def scan_models(self):
        """pi0.5_trained 폴더에서 모델 스캔"""
        self.available_models = []
        if os.path.exists(self.models_dir):
            for folder in os.listdir(self.models_dir):
                folder_path = os.path.join(self.models_dir, folder)
                if os.path.isdir(folder_path):
                    config_path = os.path.join(folder_path, "config.json")
                    model_path = os.path.join(folder_path, "model.safetensors")
                    if os.path.exists(config_path) and os.path.exists(model_path):
                        self.available_models.append(folder)

        self.log(f"[모델] {len(self.available_models)}개 모델 발견: {self.available_models}")

    def load_baseline(self):
        """베이스라인 자세 로드"""
        try:
            if os.path.exists(self.baseline_file):
                with open(self.baseline_file, 'r') as f:
                    data = json.load(f)
                if 'servos' in data:
                    for servo_id, servo_data in data['servos'].items():
                        self.baseline_positions[int(servo_id)] = servo_data.get('position', 2048)
                self.log(f"[베이스라인] 로드 완료: {len(self.baseline_positions)}개 관절")
        except Exception as e:
            self.log(f"[베이스라인] 로드 실패: {e}")

    def create_ui(self):
        """UI 생성"""
        # 메인 프레임
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 스타일 설정 (한글 폰트)
        style = ttk.Style()
        korean_font = 'NanumSquare'
        style.configure('Title.TLabel', font=(korean_font, 16, 'bold'))
        style.configure('Status.TLabel', font=(korean_font, 12))
        style.configure('Big.TButton', font=(korean_font, 12, 'bold'), padding=10)
        style.configure('TLabel', font=(korean_font, 10))
        style.configure('TButton', font=(korean_font, 10))
        style.configure('TLabelframe.Label', font=(korean_font, 10))

        # ========== 상단: 제어 패널 ==========
        control_frame = ttk.LabelFrame(main_frame, text="제어 패널", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # 모델 선택
        model_frame = ttk.Frame(control_frame)
        model_frame.pack(fill=tk.X, pady=5)

        ttk.Label(model_frame, text="모델 선택:", font=('Arial', 11)).pack(side=tk.LEFT, padx=(0, 10))

        self.model_combo = ttk.Combobox(model_frame, textvariable=self.selected_model,
                                         values=self.available_models, width=30, state='readonly')
        self.model_combo.pack(side=tk.LEFT, padx=(0, 10))
        if self.available_models:
            self.model_combo.current(0)

        ttk.Button(model_frame, text="새로고침", command=self.refresh_models).pack(side=tk.LEFT, padx=5)
        ttk.Button(model_frame, text="모델 로드", command=self.load_model).pack(side=tk.LEFT, padx=5)

        self.model_status_label = ttk.Label(model_frame, text="모델 미로드", foreground='gray')
        self.model_status_label.pack(side=tk.LEFT, padx=20)

        # 연결 상태
        conn_frame = ttk.Frame(control_frame)
        conn_frame.pack(fill=tk.X, pady=5)

        ttk.Label(conn_frame, text="로봇 연결:", font=('Arial', 11)).pack(side=tk.LEFT, padx=(0, 10))

        self.port_combo = ttk.Combobox(conn_frame, width=20, state='readonly')
        self.port_combo.pack(side=tk.LEFT, padx=(0, 10))
        self.refresh_ports()

        ttk.Button(conn_frame, text="포트 새로고침", command=self.refresh_ports).pack(side=tk.LEFT, padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="연결", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.conn_status_label = ttk.Label(conn_frame, text="연결 안됨", foreground='red')
        self.conn_status_label.pack(side=tk.LEFT, padx=20)

        # ========== 중앙: 카메라 + 상태 ==========
        middle_frame = ttk.Frame(main_frame)
        middle_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # 카메라 프레임
        camera_frame = ttk.LabelFrame(middle_frame, text="카메라 (Top / Wrist Left / Wrist Right)", padding=10)
        camera_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        # 카메라 재검색 버튼
        cam_btn_frame = ttk.Frame(camera_frame)
        cam_btn_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Button(cam_btn_frame, text="카메라 재검색", command=self._refresh_cameras, width=15).pack(side=tk.LEFT)

        cam_grid = ttk.Frame(camera_frame)
        cam_grid.pack(fill=tk.BOTH, expand=True)

        for i, name in enumerate(self.camera_names):
            frame = ttk.Frame(cam_grid)
            frame.grid(row=0, column=i, padx=5, pady=5)

            ttk.Label(frame, text=name, font=('Arial', 10, 'bold')).pack()
            canvas = tk.Canvas(frame, width=280, height=210, bg='#333')
            canvas.pack()
            self.camera_canvases[name] = canvas

        # 상태 프레임
        status_frame = ttk.LabelFrame(middle_frame, text="상태", padding=10)
        status_frame.pack(side=tk.RIGHT, fill=tk.Y, ipadx=20)

        self.status_labels = {}
        status_items = [
            ('model', '모델'),
            ('robot', '로봇'),
            ('inference', '추론'),
            ('fps', 'FPS'),
            ('action', '액션'),
        ]

        for key, label in status_items:
            row = ttk.Frame(status_frame)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=f"{label}:", width=10).pack(side=tk.LEFT)
            self.status_labels[key] = ttk.Label(row, text="-", width=20)
            self.status_labels[key].pack(side=tk.LEFT)

        # ========== Language Command ==========
        command_frame = ttk.LabelFrame(main_frame, text="Language Command", padding=10)
        command_frame.pack(fill=tk.X, pady=(0, 10))

        # Input row
        input_row = ttk.Frame(command_frame)
        input_row.pack(fill=tk.X)

        # Command input field
        self.command_entry = ttk.Entry(input_row, width=60, font=('NanumSquare', 11))
        self.command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.command_entry.insert(0, "pick up the object")
        self.command_entry.bind('<Return>', lambda e: self.apply_command())

        # Record button
        self.record_btn = tk.Button(input_row, text="🎤 Record",
                                     command=self.start_recording, font=('NanumSquare', 10),
                                     width=8, bg='#4a4a6a', fg='white')
        self.record_btn.pack(side=tk.LEFT, padx=(0, 5))

        # Apply button
        self.apply_cmd_btn = tk.Button(input_row, text="Apply",
                                        command=self.apply_command, font=('NanumSquare', 10, 'bold'),
                                        width=6, bg='#3a7a3a', fg='white')
        self.apply_cmd_btn.pack(side=tk.LEFT)

        # Status
        self.command_status = ttk.Label(command_frame, text="Type or use 🎤 Record, then press Enter or Apply",
                                         font=('NanumSquare', 9), foreground='gray')
        self.command_status.pack(anchor=tk.W, pady=(5, 0))

        # ========== 하단: 버튼들 ==========
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)

        btn_font = ('NanumSquare', 12, 'bold')

        # 기본자세 버튼
        self.baseline_btn = tk.Button(button_frame, text="Baseline",
                                       command=self.go_to_baseline, font=btn_font,
                                       width=10, height=2)
        self.baseline_btn.pack(side=tk.LEFT, padx=10)

        # 추론 시작/중지 버튼
        self.inference_btn = tk.Button(button_frame, text="Start",
                                        command=self.toggle_inference, font=btn_font,
                                        width=10, height=2)
        self.inference_btn.pack(side=tk.LEFT, padx=10)

        # 긴급 정지
        self.stop_btn = tk.Button(button_frame, text="E-STOP",
                                   command=self.emergency_stop, font=btn_font,
                                   width=10, height=2, fg='red')
        self.stop_btn.pack(side=tk.LEFT, padx=10)

        # FPS 설정
        fps_frame = ttk.Frame(button_frame)
        fps_frame.pack(side=tk.LEFT, padx=20)
        ttk.Label(fps_frame, text="FPS:", font=('NanumSquare', 10)).pack(side=tk.LEFT, padx=2)
        self.fps_var = tk.StringVar(value="11")
        fps_combo = ttk.Combobox(fps_frame, textvariable=self.fps_var, width=5,
                                  values=['5', '10', '11', '15', '20', '30'], state='readonly')
        fps_combo.pack(side=tk.LEFT, padx=5)
        fps_combo.bind('<<ComboboxSelected>>', lambda e: self.update_fps())

        # ========== 로그 ==========
        log_frame = ttk.LabelFrame(main_frame, text="로그", padding=5)
        log_frame.pack(fill=tk.X, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=8, bg='#0d0d0d', fg='#00ff00',
                                 font=('NanumSquare', 10))
        self.log_text.pack(fill=tk.X)

        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def _init_debug_log(self):
        """디버그 로그 파일 초기화"""
        try:
            with open(self.debug_log_file, 'w') as f:
                f.write(f"=== Pi0.5 Inference Debug Log ===\n")
                f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"{'='*50}\n\n")
            print(f"[DEBUG] Log file: {self.debug_log_file}")
        except Exception as e:
            print(f"[ERROR] Failed to init debug log: {e}")

    def _write_debug_log(self, message):
        """디버그 로그 파일에 기록"""
        try:
            with open(self.debug_log_file, 'a') as f:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                f.write(f"[{timestamp}] {message}\n")
        except:
            pass

    def log(self, message):
        """로그 출력"""
        timestamp = time.strftime("%H:%M:%S")
        if hasattr(self, 'log_text'):
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
        print(f"[{timestamp}] {message}")

        # DEBUG 로그는 파일에도 저장
        if "[DEBUG]" in message:
            self._write_debug_log(message)

    def refresh_models(self):
        """모델 목록 새로고침"""
        self.scan_models()
        self.model_combo['values'] = self.available_models
        if self.available_models:
            self.model_combo.current(0)
        self.log("[모델] 목록 새로고침 완료")

    def refresh_ports(self):
        """시리얼 포트 새로고침"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            # CH340 (서보 컨트롤러) 찾기
            for port in ports:
                if 'ttyUSB' in port:
                    self.port_combo.set(port)
                    break
            else:
                self.port_combo.current(0)

    def toggle_connection(self):
        """로봇 연결 토글"""
        if self.connected:
            self.disconnect_robot()
        else:
            self.connect_robot()

    def connect_robot(self):
        """로봇 연결"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("오류", "포트를 선택하세요")
            return

        try:
            self.serial_port = serial.Serial(port, 1000000, timeout=0.1)
            self.connected = True
            self.connect_btn.configure(text="연결 해제")
            self.conn_status_label.configure(text="연결됨", foreground='green')
            self.log(f"[로봇] {port} 연결 성공")
        except Exception as e:
            self.log(f"[로봇] 연결 실패: {e}")
            messagebox.showerror("연결 실패", str(e))

    def disconnect_robot(self):
        """로봇 연결 해제"""
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.connected = False
        self.connect_btn.configure(text="연결")
        self.conn_status_label.configure(text="연결 안됨", foreground='red')
        self.log("[로봇] 연결 해제")

    def load_model(self):
        """선택된 모델 로드"""
        if not LEROBOT_AVAILABLE:
            messagebox.showerror("오류", "LeRobot이 설치되지 않았습니다")
            return

        model_name = self.selected_model.get()
        if not model_name:
            messagebox.showerror("오류", "모델을 선택하세요")
            return

        model_path = os.path.join(self.models_dir, model_name)

        try:
            self.log(f"[모델] {model_name} 로딩 중...")
            self.model_status_label.configure(text="로딩 중...", foreground='orange')
            self.root.update()

            import json
            import inspect
            from safetensors.torch import load_file
            from lerobot.configs.types import FeatureType, PolicyFeature

            # Config 로드
            with open(os.path.join(model_path, "config.json"), 'r') as f:
                config_dict = json.load(f)

            # 유효한 필드만 추출
            valid_fields = set(inspect.signature(PI05Config.__init__).parameters.keys())
            valid_fields.discard('self')
            filtered_config = {k: v for k, v in config_dict.items() if k in valid_fields}

            # input_features와 output_features를 PolicyFeature 객체로 변환
            if 'input_features' in filtered_config:
                new_input_features = {}
                for key, val in filtered_config['input_features'].items():
                    feature_type = FeatureType[val['type']]
                    new_input_features[key] = PolicyFeature(type=feature_type, shape=val['shape'])
                filtered_config['input_features'] = new_input_features

            if 'output_features' in filtered_config:
                new_output_features = {}
                for key, val in filtered_config['output_features'].items():
                    feature_type = FeatureType[val['type']]
                    new_output_features[key] = PolicyFeature(type=feature_type, shape=val['shape'])
                filtered_config['output_features'] = new_output_features

            # bfloat16으로 메모리 절약
            filtered_config['dtype'] = 'bfloat16'
            filtered_config['device'] = self.device

            # 모델 생성 및 로드
            config = PI05Config(**filtered_config)
            self.policy = PI05Policy(config)

            # 가중치 로드
            weights = load_file(os.path.join(model_path, "model.safetensors"))
            self.policy.load_state_dict(weights, strict=False)
            self.policy.to(self.device)
            self.policy.eval()
            self.policy.reset()

            # tokenizer_max_length 저장
            self.tokenizer_max_length = config.tokenizer_max_length

            self.model_loaded = True
            self.model_status_label.configure(text=f"{model_name} (bf16)", foreground='green')
            self.status_labels['model'].configure(text=model_name)

            mem_gb = torch.cuda.memory_allocated() / 1024**3
            self.log(f"[모델] {model_name} 로드 완료 (bfloat16, {mem_gb:.1f}GB)")

        except Exception as e:
            self.log(f"[모델] 로드 실패: {e}")
            self.model_status_label.configure(text="로드 실패", foreground='red')
            messagebox.showerror("모델 로드 실패", str(e))

    def start_recording(self):
        """음성 녹음 시작"""
        if not SPEECH_AVAILABLE:
            messagebox.showerror("오류", "SpeechRecognition이 설치되지 않았습니다.\npip install SpeechRecognition pyaudio")
            return

        if self.is_recording:
            return

        self.is_recording = True
        self.record_btn.configure(text="🔴 녹음 중...", bg='#aa3333')
        self.log("[음성] 녹음 시작... 말씀하세요")

        # 별도 스레드에서 녹음
        threading.Thread(target=self._record_audio, daemon=True).start()

    def _record_audio(self):
        """음성 녹음 및 인식 (스레드)"""
        try:
            with sr.Microphone() as source:
                self.speech_recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.speech_recognizer.listen(source, timeout=10, phrase_time_limit=10)

            self.log("[음성] 인식 중...")

            # Google Speech Recognition (영어)
            try:
                # 먼저 영어로 인식 시도
                text = self.speech_recognizer.recognize_google(audio, language='en-US')
                self.log(f"[음성] 영어 인식: {text}")
            except:
                # 영어 실패 시 한국어로 시도 후 번역
                try:
                    text_ko = self.speech_recognizer.recognize_google(audio, language='ko-KR')
                    self.log(f"[음성] 한국어 인식: {text_ko}")

                    if TRANSLATE_AVAILABLE:
                        result = self.translator.translate(text_ko, src='ko', dest='en')
                        text = result.text
                        self.log(f"[음성] 번역됨: {text}")
                    else:
                        text = text_ko
                        self.log("[음성] 번역기 없음, 원문 사용")
                except Exception as e:
                    self.log(f"[음성] 인식 실패: {e}")
                    text = None

            # UI 업데이트 (메인 스레드)
            if text:
                self.root.after(0, lambda t=text: self._update_command_ui(t))

        except sr.WaitTimeoutError:
            self.log("[음성] 타임아웃 - 음성이 감지되지 않음")
        except Exception as e:
            self.log(f"[음성] 오류: {e}")
        finally:
            self.is_recording = False
            self.root.after(0, lambda: self.record_btn.configure(text="🎤 녹음", bg='#4a4a6a'))

    def _update_command_ui(self, text):
        """명령 UI 업데이트"""
        self.command_entry.delete(0, tk.END)
        self.command_entry.insert(0, text)
        self.apply_command()

    def apply_command(self):
        """입력된 명령을 토큰화하여 적용"""
        command = self.command_entry.get().strip()
        if not command:
            messagebox.showwarning("경고", "명령을 입력하세요")
            return

        self.current_command = command
        self.log(f"[명령] 적용: {command}")

        # 토큰화 (모델이 로드된 경우)
        if self.model_loaded and self.policy:
            try:
                self._tokenize_command(command)
                self.command_status.configure(text=f"✓ {command[:30]}...", foreground='green')
            except Exception as e:
                self.log(f"[명령] 토큰화 실패: {e}")
                self.command_status.configure(text="토큰화 실패", foreground='red')
        else:
            self.command_status.configure(text=f"대기: {command[:20]}...", foreground='orange')
            self.log("[명령] 모델 로드 후 토큰화됩니다")

    def _tokenize_command(self, command):
        """명령을 Pi0.5 토큰으로 변환"""
        if not self.policy:
            return

        try:
            from transformers import AutoTokenizer

            # 캐시된 토크나이저 사용 또는 로드
            if not hasattr(self, '_tokenizer') or self._tokenizer is None:
                self._tokenizer = AutoTokenizer.from_pretrained(
                    'google/paligemma-3b-pt-224',
                    trust_remote_code=True
                )
                self.log("[Tokenizer] PaliGemma tokenizer loaded")

            encoded = self._tokenizer(
                command,
                padding='max_length',
                truncation=True,
                max_length=self.tokenizer_max_length,
                return_tensors='pt'
            )
            self.language_tokens = encoded['input_ids'].to(self.device)
            self.language_attention_mask = encoded['attention_mask'].bool().to(self.device)
            self.log(f"[Command] Tokenized: {command[:40]}...")

        except Exception as e:
            self.log(f"[Command] Tokenization error: {e}")
            self.language_tokens = None
            self.language_attention_mask = None

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

    def _detect_cameras(self):
        """연결된 카메라 자동 감지 (USB 포트 경로 기반) - v7 방식"""
        cameras = []

        def safe_log(msg):
            try:
                self.log(msg)
            except:
                print(msg)

        saved_config = self._load_camera_config()
        internal_keywords = ['FHD Camera', 'IR Camera', 'Integrated', 'Built-in']
        external_keywords = ['C270', 'Logitech', 'Orbbec', 'Gemini', 'Astra', 'Dabai']

        def get_usb_port_path(video_idx):
            try:
                real_path = os.path.realpath(f"/sys/class/video4linux/video{video_idx}")
                parts = real_path.split('/')
                usb_ports = []
                for part in parts:
                    if part and part[0].isdigit() and '-' in part and ':' not in part:
                        usb_ports.append(part)
                if usb_ports:
                    return max(usb_ports, key=len)
            except:
                pass
            return ""

        safe_log("[카메라] 자동 감지 시작...")

        found_cameras = []
        for i in range(30):
            name_path = f"/sys/class/video4linux/video{i}/name"
            if os.path.exists(name_path):
                try:
                    with open(name_path, 'r') as f:
                        name = f.read().strip()

                    is_internal = any(kw in name for kw in internal_keywords)
                    if is_internal:
                        continue

                    is_external = any(kw in name for kw in external_keywords)
                    if not is_external:
                        continue

                    usb_port = get_usb_port_path(i)

                    if 'C270' in name or 'Logitech' in name:
                        cam_type = 'C270'
                    elif 'Orbbec' in name or 'Gemini' in name or 'Astra' in name or 'Dabai' in name:
                        cam_type = 'Gemini'
                    else:
                        cam_type = 'Unknown'

                    found_cameras.append({
                        'device': f'/dev/video{i}',
                        'video_idx': i,
                        'name': cam_type,
                        'full_name': name,
                        'usb_port': usb_port,
                        'cam_type': cam_type
                    })
                    safe_log(f"  발견: video{i} = {name} (USB: {usb_port})")
                except:
                    pass

        # USB 포트별로 그룹화
        cameras_by_port = {}
        for cam in found_cameras:
            port = cam['usb_port']
            if port not in cameras_by_port:
                cameras_by_port[port] = []
            cameras_by_port[port].append(cam)

        unique_cameras = []
        for port, cams in cameras_by_port.items():
            cams.sort(key=lambda c: c['video_idx'])
            representative = cams[0]
            unique_cameras.append(representative)
            safe_log(f"  USB {port}: {representative['cam_type']} -> video{representative['video_idx']} 선택 ({len(cams)}개 중)")

        gemini_cams = [c for c in unique_cameras if c['cam_type'] == 'Gemini']
        c270_cams = [c for c in unique_cameras if c['cam_type'] == 'C270']

        safe_log(f"[카메라] 물리 카메라: Gemini {len(gemini_cams)}개, C270 {len(c270_cams)}개")

        # 저장된 USB 포트 설정 사용
        if saved_config and 'port_roles' in saved_config:
            safe_log("[카메라] 저장된 USB 포트 설정 사용")
            port_roles = saved_config['port_roles']

            for port, role in port_roles.items():
                port_cams = [c for c in found_cameras if c['usb_port'] == port]
                if not port_cams:
                    continue

                port_cams.sort(key=lambda c: c['video_idx'])

                if role == 'top' and len(port_cams) > 0:
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
                    cam = port_cams[0]
                    cam['role'] = role
                    cameras.append(cam)
                    safe_log(f"  {role}: video{cam['video_idx']} (USB: {port})")

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

        gemini_rgb = None
        if gemini_cams:
            best_score = -1
            best_cam = None
            for cam in gemini_cams:
                vid = cam['video_idx']
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
                                best_cam = cam
                except:
                    pass

            if best_cam and best_score > 50:
                gemini_rgb = best_cam
                safe_log(f"[카메라] Gemini RGB 자동 선택: video{best_cam['video_idx']} (점수={best_score:.0f})")
            elif gemini_cams:
                gemini_rgb = gemini_cams[0]

        c270_cams.sort(key=lambda c: c['usb_port'])

        if gemini_rgb:
            gemini_rgb['role'] = 'top'
            cameras.append(gemini_rgb)

        for idx, cam in enumerate(c270_cams[:2]):
            if idx == 0:
                cam['role'] = 'wrist_right'
            else:
                cam['role'] = 'wrist_left'
            cameras.append(cam)

        for i, cam in enumerate(cameras):
            role = cam.get('role', 'unknown')
            safe_log(f"[카메라] {i+1}번: {cam['name']} (video{cam['video_idx']}) - {role}")

        return cameras

    def _refresh_cameras(self):
        """카메라 재검색"""
        self.log("[카메라] 재검색 시작...")

        # 기존 카메라 스레드 중지
        self.stop_camera = True
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)

        # 기존 카메라 해제
        for name, cap in self.cameras.items():
            try:
                cap.release()
            except:
                pass
        self.cameras.clear()
        self.camera_frames.clear()

        # 캔버스 초기화
        for name, canvas in self.camera_canvases.items():
            canvas.delete("all")

        # 카메라 스레드 재시작
        self.start_camera_thread()
        self.log("[카메라] 재검색 완료")

    def start_camera_thread(self):
        """카메라 스레드 시작"""
        self.stop_camera = False
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

    def camera_loop(self):
        """카메라 캡처 루프 - v7 방식 자동 감지"""
        # v7 방식 카메라 감지
        detected = self._detect_cameras()

        # 역할별로 카메라 열기
        role_to_name = {'top': 'top', 'wrist_right': 'wrist_right', 'wrist_left': 'wrist_left'}

        for cam in detected:
            role = cam.get('role')
            if role in role_to_name:
                name = role_to_name[role]
                try:
                    cap = cv2.VideoCapture(cam['video_idx'])
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.cameras[name] = cap
                        self.log(f"[카메라] {name} (video{cam['video_idx']}) 열림")
                except Exception as e:
                    self.log(f"[카메라] {name} 열기 실패: {e}")

        while not self.stop_camera:
            for name, cap in self.cameras.items():
                try:
                    ret, frame = cap.read()
                    if ret:
                        self.camera_frames[name] = frame
                        self.update_camera_canvas(name, frame)
                except:
                    pass
            time.sleep(0.033)  # ~30fps

    def update_camera_canvas(self, name, frame):
        """카메라 캔버스 업데이트"""
        try:
            canvas = self.camera_canvases.get(name)
            if canvas:
                # 리사이즈
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb, (280, 210))

                img = Image.fromarray(frame_resized)
                photo = ImageTk.PhotoImage(image=img)

                canvas.create_image(0, 0, anchor=tk.NW, image=photo)
                canvas.image = photo
        except:
            pass

    def go_to_baseline(self):
        """기본자세로 이동 (v7 방식 - 엘보우 안전 동작 포함)"""
        if not self.connected:
            messagebox.showwarning("경고", "로봇이 연결되지 않았습니다")
            return

        # ACC=6 설정 (기본 가속)
        self.apply_all_acceleration(6)
        time.sleep(0.1)

        # 1단계: 기본자세로 이동
        self.log("[1/3] 기본 자세로 이동 중...")
        for joint_id in self.joint_ids:
            if joint_id in self.baseline_positions:
                position = self.baseline_positions[joint_id]
                self.send_servo_command(joint_id, position)
                time.sleep(0.02)

        time.sleep(0.5)  # 잠시 대기

        # 2단계: 엘보우 안전 위치로 이동 (L +300, R -300)
        self.log("[2/3] 엘보우 안전 위치로 이동...")
        left_elbow_id = 24
        right_elbow_id = 14

        left_baseline = self.baseline_positions.get(left_elbow_id, 2048)
        right_baseline = self.baseline_positions.get(right_elbow_id, 2048)

        left_target = min(4095, left_baseline + 300)
        right_target = max(0, right_baseline - 300)

        self.send_servo_command(left_elbow_id, left_target)
        self.send_servo_command(right_elbow_id, right_target)

        time.sleep(0.5)  # 잠시 대기

        # 3단계: 다시 기본자세로 복귀
        self.log("[3/3] 기본 자세로 복귀...")
        self.send_servo_command(left_elbow_id, left_baseline)
        self.send_servo_command(right_elbow_id, right_baseline)

        self.log("[기본자세] 이동 완료 (엘보우 안전 동작 포함)")

    def send_servo_command(self, servo_id, position, time_ms=100):
        """서보 명령 전송"""
        if not self.serial_port or not self.connected:
            return

        try:
            position = max(0, min(4095, int(position)))
            time_val = max(0, min(30000, int(time_ms)))

            # STS 프로토콜
            cmd = bytes([
                0xFF, 0xFF,  # 헤더
                servo_id,
                7,  # 길이
                0x03,  # WRITE
                0x2A,  # 목표 위치 레지스터
                position & 0xFF,
                (position >> 8) & 0xFF,
                time_val & 0xFF,
                (time_val >> 8) & 0xFF,
            ])

            # 체크섬
            checksum = (~sum(cmd[2:]) & 0xFF)
            cmd = cmd + bytes([checksum])

            self.serial_port.write(cmd)
        except Exception as e:
            self.log(f"[서보] 명령 전송 실패: {e}")

    def set_servo_acceleration(self, servo_id, acc_value):
        """서보 ACC(가속도) 설정 (레지스터 0x29)"""
        if not self.serial_port or not self.connected:
            return False

        try:
            actual_acc = max(0, min(254, int(acc_value)))
            packet = [0xFF, 0xFF, servo_id, 0x04, 0x03, 0x29, actual_acc]
            checksum = (~sum(packet[2:]) % 256) & 0xFF
            self.serial_port.write(bytes(packet + [checksum]))
            return True
        except Exception as e:
            self.log(f"[서보] ACC 설정 실패: {e}")
            return False

    def apply_all_acceleration(self, acc_value=6, silent=False):
        """모든 서보에 ACC 적용"""
        if not self.connected:
            return

        if not silent:
            self.log(f"[ACC] 모든 서보에 ACC={acc_value} 적용 중...")
        for joint_id in self.joint_ids:
            self.set_servo_acceleration(joint_id, acc_value)
            time.sleep(0.01)
        if not silent:
            self.log(f"[ACC] 적용 완료")

    def read_servo_position(self, servo_id):
        """서보 현재 위치 읽기 (STS3215 프로토콜) - 빠른 버전"""
        if not self.serial_port or not self.connected:
            return None

        try:
            # 임시로 timeout 설정 (읽기용)
            old_timeout = self.serial_port.timeout
            self.serial_port.timeout = 0.02  # 20ms timeout

            packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02]
            checksum = (~sum(packet[2:]) % 256) & 0xFF

            self.serial_port.reset_input_buffer()
            self.serial_port.write(bytes(packet + [checksum]))

            response = self.serial_port.read(8)  # 정확히 8바이트만

            self.serial_port.timeout = old_timeout  # timeout 복원

            if len(response) >= 8:
                pos_l = response[5]
                pos_h = response[6]
                position = pos_l + (pos_h << 8)
                return position
            return None
        except Exception as e:
            return None

    def read_all_positions(self):
        """모든 서보 현재 위치 읽기"""
        positions = {}
        for joint_id in self.joint_ids:
            pos = self.read_servo_position(joint_id)
            if pos is not None:
                positions[joint_id] = pos
            else:
                # 읽기 실패 시 베이스라인 사용
                positions[joint_id] = self.baseline_positions.get(joint_id, 2048)
        return positions

    def toggle_inference(self):
        """추론 시작/중지"""
        if self.is_inferencing:
            self.stop_inference()
        else:
            self.start_inference()

    def start_inference(self):
        """추론 시작"""
        if not self.model_loaded:
            messagebox.showwarning("경고", "모델을 먼저 로드하세요")
            return

        if not self.connected:
            messagebox.showwarning("경고", "로봇을 먼저 연결하세요")
            return

        self.is_inferencing = True
        self.inference_btn.configure(text="Stop")
        self.status_labels['inference'].configure(text="실행 중", foreground='green')
        self.log("[추론] 시작")

        self.policy.reset()
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

    def stop_inference(self):
        """추론 중지"""
        self.is_inferencing = False
        self.inference_btn.configure(text="Start")
        self.status_labels['inference'].configure(text="중지됨")
        self.log("[추론] 중지")

    def update_fps(self):
        """FPS 업데이트"""
        try:
            self.inference_fps = int(self.fps_var.get())
            self.log(f"[FPS] Set to {self.inference_fps}")
        except:
            pass

    def inference_loop(self):
        """추론 루프"""
        interval = 1.0 / self.inference_fps
        inference_count = 0
        fps_history = []
        start_total = time.time()
        last_acc_time = 0  # 마지막 ACC 적용 시간

        while self.is_inferencing:
            start_time = time.time()

            # 1초마다 ACC=6 적용 (silent=True로 로그 억제)
            if time.time() - last_acc_time >= 1.0:
                self.apply_all_acceleration(6, silent=True)
                last_acc_time = time.time()

            try:
                # 관측값 준비
                observation = self.prepare_observation()

                if observation is not None:
                    # 추론 (매번 새로 예측하도록 큐 초기화)
                    with torch.no_grad():
                        # action queue 비우기 - 매번 새 예측 강제
                        self.policy._action_queue.clear()
                        action = self.policy.select_action(observation)

                    # 액션 적용
                    self.apply_action(action)

                    # 성능 측정
                    inference_time = time.time() - start_time
                    current_fps = 1.0 / inference_time if inference_time > 0 else 0
                    fps_history.append(current_fps)
                    if len(fps_history) > 30:  # 최근 30개만 유지
                        fps_history.pop(0)
                    avg_fps = sum(fps_history) / len(fps_history)
                    inference_count += 1

                    # 상태 업데이트
                    self.root.after(0, lambda f=current_fps, a=avg_fps, c=inference_count:
                        self.status_labels['fps'].configure(
                            text=f"{f:.1f} (avg: {a:.1f}) #{c}"))

            except Exception as e:
                self.log(f"[추론] 오류: {e}")

            # FPS 유지
            elapsed = time.time() - start_time
            if elapsed < interval:
                time.sleep(interval - elapsed)

        # 테스트 종료 시 결과 출력
        total_time = time.time() - start_total
        if inference_count > 0:
            self.log(f"[성능] 총 {inference_count}회 추론, {total_time:.1f}초, 평균 {inference_count/total_time:.1f} FPS")

    def prepare_observation(self):
        """관측값 준비"""
        try:
            observation = {}

            # 이미지 처리
            camera_status = []
            for name in self.camera_names:
                frame = self.camera_frames.get(name)
                if frame is not None:
                    # 224x224로 리사이즈
                    frame_resized = cv2.resize(frame, (224, 224))
                    frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

                    # 정규화 및 텐서 변환 (bfloat16으로 - 모델과 dtype 일치)
                    frame_normalized = frame_rgb.astype(np.float32) / 255.0
                    frame_tensor = torch.from_numpy(frame_normalized).permute(2, 0, 1)  # HWC -> CHW
                    frame_tensor = frame_tensor.to(dtype=torch.bfloat16, device=self.device)

                    key = f"observation.images.{name}"
                    observation[key] = frame_tensor.unsqueeze(0)
                    camera_status.append(f"{name}:OK")
                else:
                    # 더미 이미지 (검은 화면)
                    dummy = torch.zeros(1, 3, 224, 224, dtype=torch.bfloat16, device=self.device)
                    observation[f"observation.images.{name}"] = dummy
                    camera_status.append(f"{name}:DUMMY")

            # 디버그: 카메라 상태 출력
            self.log(f"[DEBUG] Cameras: {camera_status}")

            # 상태 (현재 관절 위치 - QUANTILES 정규화)
            # 모든 서보 현재 위치 읽기 (closed-loop 제어)
            current_positions = self.read_all_positions()

            state = np.zeros(16, dtype=np.float32)
            for i, joint_id in enumerate(self.joint_ids):
                # 현재 위치 사용 (읽기 실패 시 베이스라인)
                pos = current_positions.get(joint_id, self.baseline_positions.get(joint_id, 2048))
                # 1단계: LINEAR 정규화 (0-4095 -> roughly -1 to 1)
                linear = (pos - 2048) / 2048.0
                # 2단계: QUANTILES 정규화 (training data의 min/max 기준으로 -1 to 1)
                range_val = self.state_max[i] - self.state_min[i]
                if range_val > 0.001:  # 0으로 나누기 방지
                    quantile = 2.0 * (linear - self.state_min[i]) / range_val - 1.0
                else:
                    quantile = 0.0
                # 클리핑 (-1 ~ 1)
                state[i] = np.clip(quantile, -1.0, 1.0)

            # 디버그: STATE 정규화 값 출력
            self.log(f"[DEBUG] State QUANTILES (R arm): {[f'{v:.3f}' for v in state[:8]]}")
            self.log(f"[DEBUG] State QUANTILES (L arm): {[f'{v:.3f}' for v in state[8:16]]}")

            observation["observation.state"] = torch.from_numpy(state).unsqueeze(0).to(dtype=torch.bfloat16, device=self.device)

            # 언어 토큰 (Pi0.5 필수) - 음성 명령 사용
            tokenizer_max_length = getattr(self, 'tokenizer_max_length', 200)
            if self.language_tokens is not None and self.language_attention_mask is not None:
                # 토큰화된 명령 사용
                observation["observation.language.tokens"] = self.language_tokens
                observation["observation.language.attention_mask"] = self.language_attention_mask
            else:
                # 명령 없으면 빈 토큰
                observation["observation.language.tokens"] = torch.zeros(1, tokenizer_max_length, dtype=torch.long).to(self.device)
                observation["observation.language.attention_mask"] = torch.zeros(1, tokenizer_max_length, dtype=torch.bool).to(self.device)

            return observation

        except Exception as e:
            self.log(f"[관측] 준비 실패: {e}")
            return None

    def apply_action(self, action):
        """액션 적용"""
        try:
            if action is None:
                return

            action_np = action.cpu().numpy()

            # 디버그: 모델 출력 형태
            self.log(f"[DEBUG] Action shape: {action_np.shape}")

            # 평탄화
            action_flat = action_np.flatten()

            # 첫 번째 액션만 사용 (chunk에서)
            if len(action_flat) >= 16:
                action_values = action_flat[:16]
            else:
                action_values = action_flat

            # 디버그: QUANTILES 정규화된 액션값 (처음 8개)
            self.log(f"[DEBUG] Action QUANTILES (R arm): {[f'{v:.3f}' for v in action_values[:8]]}")

            # QUANTILES 역정규화된 위치 계산
            positions = []
            linear_values = []
            # 반전 필요한 서보 ID (오른쪽 그리퍼만 - v7 기준)
            inverted_servos = [41]
            # 오른팔 서보 ID (고정 - 명령 안 보냄)
            right_arm_ids = [11, 12, 13, 14, 15, 16, 17, 41]

            for i, joint_id in enumerate(self.joint_ids):
                if i < len(action_values):
                    quantile = action_values[i]
                    # 1단계: QUANTILES -> LINEAR 역정규화
                    range_val = self.action_max[i] - self.action_min[i]
                    linear = (quantile + 1.0) * range_val / 2.0 + self.action_min[i]
                    linear_values.append(linear)
                    # 2단계: LINEAR -> 원시 위치 역정규화
                    position = int(linear * 2048 + 2048)
                    position = max(0, min(4095, position))

                    # 서보 방향 반전
                    if joint_id in inverted_servos:
                        position = 4095 - position

                    positions.append(position)

                    # 오른팔은 명령 안 보냄 (기본자세 고정)
                    if joint_id not in right_arm_ids:
                        self.send_servo_command(joint_id, position, time_ms=50)

            # 디버그: LINEAR 역정규화 값
            self.log(f"[DEBUG] Action LINEAR (R arm): {[f'{v:.3f}' for v in linear_values[:8]]}")
            self.log(f"[DEBUG] Action LINEAR (L arm): {[f'{v:.3f}' for v in linear_values[8:16]]}")
            # 디버그: 최종 서보 위치
            self.log(f"[DEBUG] Positions (R arm): {positions[:8]}")
            self.log(f"[DEBUG] Positions (L arm): {positions[8:16]}")

            # 상태 업데이트
            self.root.after(0, lambda av=action_values: self.status_labels['action'].configure(
                text=f"[{av[0]:.2f}, {av[1]:.2f}, ...]"))

        except Exception as e:
            self.log(f"[액션] 적용 실패: {e}")

    def emergency_stop(self):
        """긴급 정지"""
        self.is_inferencing = False
        self.inference_btn.configure(text="Start")
        self.status_labels['inference'].configure(text="긴급 정지", foreground='red')
        self.log("[긴급정지] 모든 동작 중지")

        # 기본자세로 복귀 (연결된 경우에만)
        if self.connected:
            self.go_to_baseline()

    def on_closing(self):
        """종료 처리"""
        self.is_inferencing = False
        self.stop_camera = True

        # 카메라 해제
        for cap in self.cameras.values():
            cap.release()

        # 로봇 연결 해제
        if self.serial_port:
            self.serial_port.close()

        self.root.destroy()

    def run(self):
        """GUI 실행"""
        self.log("Pi0.5 Inference GUI 시작")
        self.log(f"Device: {self.device}")
        self.log(f"LeRobot 사용 가능: {LEROBOT_AVAILABLE}")
        self.root.mainloop()


def main():
    gui = Pi05InferenceGUI()
    gui.run()


if __name__ == "__main__":
    main()
