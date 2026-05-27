#!/usr/bin/env python3
"""
그리퍼 전류/부하 모니터링 테스트 GUI
- 그리퍼 서보의 전류(Load) 값을 실시간 모니터링
- 데이터를 txt 파일로 저장
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
from pathlib import Path

class GripperMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("그리퍼 전류 모니터링 테스트")
        self.root.geometry("800x600")

        # 시리얼 연결
        self.ser = None
        self.connected = False
        self.monitoring = False
        self.monitor_thread = None

        # 데이터 저장
        self.log_file = None
        self.data_buffer = []

        # 그리퍼 서보 ID
        self.right_gripper_id = 41  # 오른손 그리퍼
        self.left_gripper_id = 31   # 왼손 그리퍼

        # 현재 값
        self.right_load = tk.StringVar(value="--")
        self.right_position = tk.StringVar(value="--")
        self.right_voltage = tk.StringVar(value="--")
        self.left_load = tk.StringVar(value="--")
        self.left_position = tk.StringVar(value="--")
        self.left_voltage = tk.StringVar(value="--")

        self.setup_ui()

    def setup_ui(self):
        # 연결 프레임
        conn_frame = ttk.LabelFrame(self.root, text="연결", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(conn_frame, text="포트:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(conn_frame, width=20)
        self.port_combo.pack(side=tk.LEFT, padx=5)

        ttk.Button(conn_frame, text="새로고침", command=self.refresh_ports).pack(side=tk.LEFT, padx=5)
        self.connect_btn = ttk.Button(conn_frame, text="연결", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="연결 안됨", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        # 모니터링 프레임
        monitor_frame = ttk.LabelFrame(self.root, text="실시간 모니터링", padding=10)
        monitor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 오른손 그리퍼
        right_frame = ttk.LabelFrame(monitor_frame, text=f"오른손 그리퍼 (ID: {self.right_gripper_id})", padding=10)
        right_frame.pack(fill=tk.X, pady=5)

        ttk.Label(right_frame, text="Load (전류):", font=("Arial", 12)).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(right_frame, textvariable=self.right_load, font=("Arial", 16, "bold"), foreground="blue").grid(row=0, column=1, padx=20)

        ttk.Label(right_frame, text="Position:", font=("Arial", 12)).grid(row=0, column=2, sticky=tk.W)
        ttk.Label(right_frame, textvariable=self.right_position, font=("Arial", 14)).grid(row=0, column=3, padx=20)

        ttk.Label(right_frame, text="Voltage:", font=("Arial", 12)).grid(row=0, column=4, sticky=tk.W)
        ttk.Label(right_frame, textvariable=self.right_voltage, font=("Arial", 14)).grid(row=0, column=5, padx=20)

        # 왼손 그리퍼
        left_frame = ttk.LabelFrame(monitor_frame, text=f"왼손 그리퍼 (ID: {self.left_gripper_id})", padding=10)
        left_frame.pack(fill=tk.X, pady=5)

        ttk.Label(left_frame, text="Load (전류):", font=("Arial", 12)).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(left_frame, textvariable=self.left_load, font=("Arial", 16, "bold"), foreground="green").grid(row=0, column=1, padx=20)

        ttk.Label(left_frame, text="Position:", font=("Arial", 12)).grid(row=0, column=2, sticky=tk.W)
        ttk.Label(left_frame, textvariable=self.left_position, font=("Arial", 14)).grid(row=0, column=3, padx=20)

        ttk.Label(left_frame, text="Voltage:", font=("Arial", 12)).grid(row=0, column=4, sticky=tk.W)
        ttk.Label(left_frame, textvariable=self.left_voltage, font=("Arial", 14)).grid(row=0, column=5, padx=20)

        # Load 게이지 바
        gauge_frame = ttk.LabelFrame(monitor_frame, text="Load 게이지", padding=10)
        gauge_frame.pack(fill=tk.X, pady=5)

        ttk.Label(gauge_frame, text="오른손:").grid(row=0, column=0)
        self.right_gauge = ttk.Progressbar(gauge_frame, length=300, maximum=1000)
        self.right_gauge.grid(row=0, column=1, padx=10, pady=5)
        self.right_gauge_label = ttk.Label(gauge_frame, text="0%")
        self.right_gauge_label.grid(row=0, column=2)

        ttk.Label(gauge_frame, text="왼손:").grid(row=1, column=0)
        self.left_gauge = ttk.Progressbar(gauge_frame, length=300, maximum=1000)
        self.left_gauge.grid(row=1, column=1, padx=10, pady=5)
        self.left_gauge_label = ttk.Label(gauge_frame, text="0%")
        self.left_gauge_label.grid(row=1, column=2)

        # 컨트롤 프레임
        ctrl_frame = ttk.LabelFrame(self.root, text="컨트롤", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)

        self.monitor_btn = ttk.Button(ctrl_frame, text="모니터링 시작", command=self.toggle_monitoring)
        self.monitor_btn.pack(side=tk.LEFT, padx=5)

        self.record_btn = ttk.Button(ctrl_frame, text="녹화 시작", command=self.toggle_recording)
        self.record_btn.pack(side=tk.LEFT, padx=5)

        self.recording = False
        self.record_status = ttk.Label(ctrl_frame, text="")
        self.record_status.pack(side=tk.LEFT, padx=10)

        ttk.Label(ctrl_frame, text="샘플링 Hz:").pack(side=tk.LEFT, padx=(20, 5))
        self.hz_var = tk.StringVar(value="30")
        hz_combo = ttk.Combobox(ctrl_frame, textvariable=self.hz_var, values=["10", "20", "30", "50", "100"], width=5)
        hz_combo.pack(side=tk.LEFT)

        # 로그 프레임
        log_frame = ttk.LabelFrame(self.root, text="로그", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=10, font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(self.log_text, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)

        # 초기화
        self.refresh_ports()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            # ttyACM0 우선 선택 (로봇 포트)
            for p in ports:
                if 'ttyACM0' in p:
                    self.port_combo.set(p)
                    return
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("경고", "포트를 선택하세요")
            return

        try:
            self.ser = serial.Serial(port, 1000000, timeout=0.1)
            self.connected = True
            self.connect_btn.config(text="연결 해제")
            self.status_label.config(text=f"연결됨: {port}", foreground="green")
            self.log(f"연결 성공: {port}")
        except Exception as e:
            messagebox.showerror("오류", f"연결 실패: {e}")

    def disconnect(self):
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connected = False
        self.connect_btn.config(text="연결")
        self.status_label.config(text="연결 안됨", foreground="red")
        self.log("연결 해제됨")

    def toggle_monitoring(self):
        if self.monitoring:
            self.monitoring = False
            self.monitor_btn.config(text="모니터링 시작")
            self.log("모니터링 중지")
        else:
            if not self.connected:
                messagebox.showwarning("경고", "먼저 연결하세요")
                return
            self.monitoring = True
            self.monitor_btn.config(text="모니터링 중지")
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            self.log("모니터링 시작")

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        # 저장 파일 생성
        save_dir = Path(__file__).parent / "gripper_logs"
        save_dir.mkdir(exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = save_dir / f"gripper_log_{timestamp}.txt"

        self.log_file = open(self.log_filename, 'w')
        self.log_file.write("timestamp,right_load,right_pos,right_volt,left_load,left_pos,left_volt\n")

        self.recording = True
        self.record_btn.config(text="녹화 중지")
        self.record_status.config(text=f"녹화 중: {self.log_filename.name}", foreground="red")
        self.log(f"녹화 시작: {self.log_filename}")

    def stop_recording(self):
        self.recording = False
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.record_btn.config(text="녹화 시작")
        self.record_status.config(text=f"저장됨: {self.log_filename.name}", foreground="blue")
        self.log(f"녹화 완료: {self.log_filename}")

    def monitor_loop(self):
        """모니터링 루프"""
        while self.monitoring:
            try:
                hz = int(self.hz_var.get())
                sleep_time = 1.0 / hz

                # 오른손 그리퍼 읽기
                right_data = self.read_servo_data(self.right_gripper_id)

                # 왼손 그리퍼 읽기
                left_data = self.read_servo_data(self.left_gripper_id)

                # GUI 업데이트
                self.root.after(0, lambda r=right_data, l=left_data: self.update_display(r, l))

                # 녹화 중이면 저장
                if self.recording and self.log_file:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    line = f"{timestamp},{right_data['load']},{right_data['position']},{right_data['voltage']},"
                    line += f"{left_data['load']},{left_data['position']},{left_data['voltage']}\n"
                    self.log_file.write(line)
                    self.log_file.flush()

                time.sleep(sleep_time)

            except Exception as e:
                self.log(f"모니터링 오류: {e}")
                time.sleep(0.1)

    def read_servo_data(self, servo_id):
        """서보 데이터 읽기 (위치, 부하, 전압)"""
        data = {'position': 0, 'load': 0, 'voltage': 0}

        if not self.ser:
            return data

        try:
            # STS3215 Read 패킷: 주소 0x38부터 10바이트 읽기
            # 0x38-0x39: Present Position
            # 0x3C-0x3D: Present Speed
            # 0x3E-0x3F: Present Load
            # 0x40: Present Voltage
            # 0x41: Present Temperature

            start_addr = 0x38
            length = 10

            packet = bytearray([0xFF, 0xFF, servo_id, 0x04, 0x02, start_addr, length])
            checksum = (~(servo_id + 0x04 + 0x02 + start_addr + length)) & 0xFF
            packet.append(checksum)

            self.ser.reset_input_buffer()
            self.ser.write(packet)
            self.ser.flush()

            # 응답 읽기 (헤더 2 + ID 1 + Length 1 + Error 1 + Data N + Checksum 1)
            time.sleep(0.005)
            response = self.ser.read(50)

            if len(response) >= 15:
                # 응답 파싱
                # 헤더 찾기
                for i in range(len(response) - 14):
                    if response[i] == 0xFF and response[i+1] == 0xFF and response[i+2] == servo_id:
                        resp_length = response[i+3]
                        if resp_length >= 11:  # Error(1) + Data(10)
                            # Present Position (0x38-0x39)
                            pos_low = response[i+5]
                            pos_high = response[i+6]
                            data['position'] = pos_low | (pos_high << 8)

                            # Present Load (0x3E-0x3F) - offset 6,7 from data start
                            load_low = response[i+11]
                            load_high = response[i+12]
                            raw_load = load_low | (load_high << 8)
                            # Load는 부호 있는 값 (방향 포함)
                            if raw_load > 32767:
                                raw_load = raw_load - 65536
                            data['load'] = abs(raw_load)

                            # Present Voltage (0x40) - offset 8 from data start
                            data['voltage'] = response[i+13] / 10.0  # 0.1V 단위

                        break

        except Exception as e:
            pass

        return data

    def update_display(self, right_data, left_data):
        """GUI 표시 업데이트"""
        # 오른손
        self.right_load.set(f"{right_data['load']}")
        self.right_position.set(f"{right_data['position']}")
        self.right_voltage.set(f"{right_data['voltage']:.1f}V")

        # 왼손
        self.left_load.set(f"{left_data['load']}")
        self.left_position.set(f"{left_data['position']}")
        self.left_voltage.set(f"{left_data['voltage']:.1f}V")

        # 게이지 업데이트
        self.right_gauge['value'] = min(right_data['load'], 1000)
        self.right_gauge_label.config(text=f"{min(right_data['load'], 1000) / 10:.0f}%")

        self.left_gauge['value'] = min(left_data['load'], 1000)
        self.left_gauge_label.config(text=f"{min(left_data['load'], 1000) / 10:.0f}%")

        # 부하 임계값 경고 (예: 500 이상)
        if right_data['load'] > 500:
            self.right_gauge.config(style="red.Horizontal.TProgressbar")
        else:
            self.right_gauge.config(style="")

        if left_data['load'] > 500:
            self.left_gauge.config(style="red.Horizontal.TProgressbar")
        else:
            self.left_gauge.config(style="")

    def log(self, message):
        """로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def on_closing(self):
        """종료 처리"""
        self.monitoring = False
        if self.recording:
            self.stop_recording()
        if self.connected:
            self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = GripperMonitorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
