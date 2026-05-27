#!/usr/bin/env python3
"""
그리퍼 전류 측정 도구
- J31 (Left Gripper), J41 (Right Gripper) 개별 제어
- 실시간 전류/부하 모니터링
- 값 기록 및 TXT 저장
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import time
import threading
from datetime import datetime


class GripperCurrentTool:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("그리퍼 전류 측정 도구")
        self.root.geometry("600x800")
        self.root.resizable(False, False)

        # Serial connection
        self.ser = None
        self.connected = False

        # Gripper IDs
        self.GRIPPER_LEFT = 31
        self.GRIPPER_RIGHT = 41

        # Shoulder IDs (위치 조절용, 부하 측정 안함)
        self.SHOULDER_LEFT = 21
        self.SHOULDER_RIGHT = 11

        # STS3215 Register addresses
        self.ADDR_PRESENT_POSITION = 0x38  # 56
        self.ADDR_PRESENT_LOAD = 0x3C      # 60
        self.ADDR_PRESENT_CURRENT = 0x45   # 69
        self.ADDR_GOAL_POSITION = 0x2A     # 42

        # Current values - Grippers
        self.left_position = tk.IntVar(value=0)
        self.right_position = tk.IntVar(value=0)
        self.left_load = tk.StringVar(value="-- ")
        self.right_load = tk.StringVar(value="-- ")
        self.left_current = tk.StringVar(value="-- mA")
        self.right_current = tk.StringVar(value="-- mA")

        # Shoulder positions (부하 측정 안함)
        self.left_shoulder_pos = tk.IntVar(value=95)
        self.right_shoulder_pos = tk.IntVar(value=4000)

        # 기본자세 (rx1_baseline_v5.json 참고)
        self.BASELINE = {
            11: 4000,  # Right Shoulder
            21: 95,    # Left Shoulder
            31: 0,     # Left Gripper
            41: 0      # Right Gripper
        }

        # Recording
        self.records = []
        self.memo_text = tk.StringVar(value="")

        # Update thread
        self.running = False
        self.update_thread = None

        self.setup_ui()
        self.scan_ports()

    def setup_ui(self):
        # === Connection Frame ===
        conn_frame = ttk.LabelFrame(self.root, text="연결", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(conn_frame, text="포트:").pack(side='left')
        self.port_combo = ttk.Combobox(conn_frame, width=20, state='readonly')
        self.port_combo.pack(side='left', padx=5)

        ttk.Button(conn_frame, text="새로고침", command=self.scan_ports).pack(side='left', padx=2)
        self.connect_btn = ttk.Button(conn_frame, text="연결", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=5)

        self.status_label = ttk.Label(conn_frame, text="미연결", foreground='red')
        self.status_label.pack(side='right')

        # === Shoulder Frame (위치 조절용) ===
        shoulder_frame = ttk.LabelFrame(self.root, text="어깨 (위치 조절용)", padding=10)
        shoulder_frame.pack(fill='x', padx=10, pady=5)

        # J21 Left Shoulder
        ttk.Label(shoulder_frame, text="J21 (L):").grid(row=0, column=0, sticky='w')
        self.left_shoulder_slider = ttk.Scale(shoulder_frame, from_=0, to=4095,
                                               variable=self.left_shoulder_pos, orient='horizontal',
                                               command=lambda v: self.on_slider_change(self.SHOULDER_LEFT, v))
        self.left_shoulder_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.left_shoulder_label = ttk.Label(shoulder_frame, text="2048", width=6)
        self.left_shoulder_label.grid(row=0, column=2)

        # J11 Right Shoulder
        ttk.Label(shoulder_frame, text="J11 (R):").grid(row=1, column=0, sticky='w', pady=5)
        self.right_shoulder_slider = ttk.Scale(shoulder_frame, from_=0, to=4095,
                                                variable=self.right_shoulder_pos, orient='horizontal',
                                                command=lambda v: self.on_slider_change(self.SHOULDER_RIGHT, v))
        self.right_shoulder_slider.grid(row=1, column=1, sticky='ew', padx=5)
        self.right_shoulder_label = ttk.Label(shoulder_frame, text="2048", width=6)
        self.right_shoulder_label.grid(row=1, column=2)

        shoulder_frame.columnconfigure(1, weight=1)

        # === Left Gripper Frame ===
        left_frame = ttk.LabelFrame(self.root, text="J31 (Left Gripper)", padding=10)
        left_frame.pack(fill='x', padx=10, pady=5)

        # Position slider
        ttk.Label(left_frame, text="위치:").grid(row=0, column=0, sticky='w')
        self.left_slider = ttk.Scale(left_frame, from_=0, to=4095,
                                      variable=self.left_position, orient='horizontal',
                                      command=lambda v: self.on_slider_change(self.GRIPPER_LEFT, v))
        self.left_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.left_pos_label = ttk.Label(left_frame, text="0", width=6)
        self.left_pos_label.grid(row=0, column=2)

        # Load display
        ttk.Label(left_frame, text="부하:").grid(row=1, column=0, sticky='w', pady=5)
        self.left_load_bar = ttk.Progressbar(left_frame, length=200, mode='determinate', maximum=1000)
        self.left_load_bar.grid(row=1, column=1, sticky='ew', padx=5)
        ttk.Label(left_frame, textvariable=self.left_load, width=8).grid(row=1, column=2)

        # Current display
        ttk.Label(left_frame, text="전류:").grid(row=2, column=0, sticky='w')
        self.left_current_bar = ttk.Progressbar(left_frame, length=200, mode='determinate', maximum=2000)
        self.left_current_bar.grid(row=2, column=1, sticky='ew', padx=5)
        ttk.Label(left_frame, textvariable=self.left_current, width=8).grid(row=2, column=2)

        left_frame.columnconfigure(1, weight=1)

        # === Right Gripper Frame ===
        right_frame = ttk.LabelFrame(self.root, text="J41 (Right Gripper)", padding=10)
        right_frame.pack(fill='x', padx=10, pady=5)

        # Position slider
        ttk.Label(right_frame, text="위치:").grid(row=0, column=0, sticky='w')
        self.right_slider = ttk.Scale(right_frame, from_=0, to=4095,
                                       variable=self.right_position, orient='horizontal',
                                       command=lambda v: self.on_slider_change(self.GRIPPER_RIGHT, v))
        self.right_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.right_pos_label = ttk.Label(right_frame, text="0", width=6)
        self.right_pos_label.grid(row=0, column=2)

        # Load display
        ttk.Label(right_frame, text="부하:").grid(row=1, column=0, sticky='w', pady=5)
        self.right_load_bar = ttk.Progressbar(right_frame, length=200, mode='determinate', maximum=1000)
        self.right_load_bar.grid(row=1, column=1, sticky='ew', padx=5)
        ttk.Label(right_frame, textvariable=self.right_load, width=8).grid(row=1, column=2)

        # Current display
        ttk.Label(right_frame, text="전류:").grid(row=2, column=0, sticky='w')
        self.right_current_bar = ttk.Progressbar(right_frame, length=200, mode='determinate', maximum=2000)
        self.right_current_bar.grid(row=2, column=1, sticky='ew', padx=5)
        ttk.Label(right_frame, textvariable=self.right_current, width=8).grid(row=2, column=2)

        right_frame.columnconfigure(1, weight=1)

        # === Quick Buttons ===
        quick_frame = ttk.Frame(self.root, padding=5)
        quick_frame.pack(fill='x', padx=10)

        ttk.Button(quick_frame, text="기본자세", command=self.go_to_baseline).pack(side='left', padx=5)
        ttk.Label(quick_frame, text="  |  ").pack(side='left')
        ttk.Button(quick_frame, text="L 열기", command=lambda: self.set_gripper(31, 0)).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="L 닫기", command=lambda: self.set_gripper(31, 2000)).pack(side='left', padx=2)
        ttk.Label(quick_frame, text="  |  ").pack(side='left')
        ttk.Button(quick_frame, text="R 열기", command=lambda: self.set_gripper(41, 0)).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="R 닫기", command=lambda: self.set_gripper(41, 2000)).pack(side='left', padx=2)

        # === Record Frame ===
        record_frame = ttk.LabelFrame(self.root, text="기록", padding=10)
        record_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(record_frame, text="메모:").pack(side='left')
        ttk.Entry(record_frame, textvariable=self.memo_text, width=30).pack(side='left', padx=5)
        ttk.Button(record_frame, text="기록", command=self.record_values).pack(side='left', padx=5)

        # === Records List ===
        list_frame = ttk.LabelFrame(self.root, text="기록된 값", padding=10)
        list_frame.pack(fill='both', expand=True, padx=10, pady=5)

        # Treeview for records
        columns = ('no', 'time', 'j31_pos', 'j31_curr', 'j41_pos', 'j41_curr', 'memo')
        self.records_tree = ttk.Treeview(list_frame, columns=columns, show='headings', height=8)

        self.records_tree.heading('no', text='#')
        self.records_tree.heading('time', text='시간')
        self.records_tree.heading('j31_pos', text='J31 위치')
        self.records_tree.heading('j31_curr', text='J31 전류')
        self.records_tree.heading('j41_pos', text='J41 위치')
        self.records_tree.heading('j41_curr', text='J41 전류')
        self.records_tree.heading('memo', text='메모')

        self.records_tree.column('no', width=30)
        self.records_tree.column('time', width=70)
        self.records_tree.column('j31_pos', width=60)
        self.records_tree.column('j31_curr', width=70)
        self.records_tree.column('j41_pos', width=60)
        self.records_tree.column('j41_curr', width=70)
        self.records_tree.column('memo', width=140)

        scrollbar = ttk.Scrollbar(list_frame, orient='vertical', command=self.records_tree.yview)
        self.records_tree.configure(yscrollcommand=scrollbar.set)

        self.records_tree.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        # === Bottom Buttons ===
        btn_frame = ttk.Frame(self.root, padding=10)
        btn_frame.pack(fill='x')

        ttk.Button(btn_frame, text="선택 삭제", command=self.delete_selected).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="전체 삭제", command=self.clear_records).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="TXT 저장", command=self.save_to_txt).pack(side='right', padx=5)

    def scan_ports(self):
        """사용 가능한 시리얼 포트 검색"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            # ttyUSB0 우선 선택
            for i, p in enumerate(ports):
                if 'ttyUSB0' in p:
                    self.port_combo.current(i)
                    return
            self.port_combo.current(0)

    def toggle_connection(self):
        """연결/해제 토글"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """시리얼 연결"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("오류", "포트를 선택하세요")
            return

        try:
            self.ser = serial.Serial(port, 1000000, timeout=0.1)
            self.connected = True
            self.running = True

            self.connect_btn.config(text="해제")
            self.status_label.config(text="연결됨", foreground='green')

            # Start update thread
            self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
            self.update_thread.start()

            # Read initial positions
            self.read_initial_positions()

        except Exception as e:
            messagebox.showerror("연결 오류", str(e))

    def disconnect(self):
        """시리얼 해제"""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=1)

        if self.ser:
            self.ser.close()
            self.ser = None

        self.connected = False
        self.connect_btn.config(text="연결")
        self.status_label.config(text="미연결", foreground='red')

    def read_initial_positions(self):
        """초기 위치 읽기"""
        all_ids = [self.GRIPPER_LEFT, self.GRIPPER_RIGHT, self.SHOULDER_LEFT, self.SHOULDER_RIGHT]
        positions = self.sync_read_positions(all_ids)
        if self.GRIPPER_LEFT in positions:
            self.left_position.set(positions[self.GRIPPER_LEFT])
        if self.GRIPPER_RIGHT in positions:
            self.right_position.set(positions[self.GRIPPER_RIGHT])
        if self.SHOULDER_LEFT in positions:
            self.left_shoulder_pos.set(positions[self.SHOULDER_LEFT])
        if self.SHOULDER_RIGHT in positions:
            self.right_shoulder_pos.set(positions[self.SHOULDER_RIGHT])

    def sync_read_positions(self, servo_ids):
        """여러 서보 위치 읽기"""
        if not self.ser:
            return {}

        try:
            instruction = 0x82  # SYNC_READ
            mem_addr = self.ADDR_PRESENT_POSITION
            data_len = 0x02

            length = len(servo_ids) + 4
            packet = [0xFF, 0xFF, 0xFE, length, instruction, mem_addr, data_len]
            packet.extend(servo_ids)

            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))

            time.sleep(0.01)
            response = self.ser.read(100)

            positions = {}
            idx = 0
            for servo_id in servo_ids:
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

    def read_servo_current(self, servo_id):
        """단일 서보 실제 전류 읽기 (주소 69-70) - 실제 부하 측정"""
        if not self.ser:
            return None

        try:
            # Read instruction: FF FF [ID] 04 02 [ADDR] [LEN] [CHECKSUM]
            # Present Current = 0x45 (69)
            packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x45, 0x02]
            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))

            time.sleep(0.005)
            response = self.ser.read(20)

            # Parse response: FF FF [ID] [LEN] [ERR] [DATA_L] [DATA_H] [CHECKSUM]
            for i in range(len(response) - 7):
                if response[i] == 0xFF and response[i+1] == 0xFF and response[i+2] == servo_id:
                    curr_l = response[i+5]
                    curr_h = response[i+6]
                    current = curr_l + (curr_h << 8)
                    # Current is signed 16-bit value (mA)
                    if current > 32767:
                        current = current - 65536
                    return current

            return None

        except Exception as e:
            return None

    def send_position(self, servo_id, position):
        """서보 위치 명령 전송"""
        if not self.ser:
            return

        try:
            position = max(0, min(4095, int(position)))
            pos_l = position & 0xFF
            pos_h = (position >> 8) & 0xFF

            # Write instruction: FF FF [ID] 05 03 [ADDR] [DATA_L] [DATA_H] [CHECKSUM]
            packet = [0xFF, 0xFF, servo_id, 0x05, 0x03, self.ADDR_GOAL_POSITION, pos_l, pos_h]
            checksum = (~sum(packet[2:]) % 256) & 0xFF
            packet.append(checksum)

            self.ser.write(bytes(packet))

        except Exception as e:
            pass

    def on_slider_change(self, servo_id, value):
        """슬라이더 변경 시 호출"""
        if not self.connected:
            return

        position = int(float(value))
        self.send_position(servo_id, position)

        # Update label
        if servo_id == self.GRIPPER_LEFT:
            self.left_pos_label.config(text=str(position))
        elif servo_id == self.GRIPPER_RIGHT:
            self.right_pos_label.config(text=str(position))
        elif servo_id == self.SHOULDER_LEFT:
            self.left_shoulder_label.config(text=str(position))
        elif servo_id == self.SHOULDER_RIGHT:
            self.right_shoulder_label.config(text=str(position))

    def set_gripper(self, servo_id, position):
        """그리퍼 위치 직접 설정"""
        if servo_id == self.GRIPPER_LEFT:
            self.left_position.set(position)
        else:
            self.right_position.set(position)
        self.send_position(servo_id, position)

    def go_to_baseline(self):
        """기본자세로 이동"""
        if not self.connected:
            return

        # 모든 서보를 기본자세로
        for servo_id, position in self.BASELINE.items():
            self.send_position(servo_id, position)

        # UI 슬라이더 업데이트
        self.left_shoulder_pos.set(self.BASELINE[21])
        self.right_shoulder_pos.set(self.BASELINE[11])
        self.left_position.set(self.BASELINE[31])
        self.right_position.set(self.BASELINE[41])

    def update_loop(self):
        """실시간 업데이트 루프"""
        while self.running:
            try:
                # Read actual current (실제 전류)
                left_curr = self.read_servo_current(self.GRIPPER_LEFT)
                right_curr = self.read_servo_current(self.GRIPPER_RIGHT)

                # Update UI (thread-safe)
                self.root.after(0, lambda l=left_curr, r=right_curr: self.update_display(l, r))

            except Exception as e:
                pass

            time.sleep(0.1)  # 10Hz update

    def update_display(self, left_curr, right_curr):
        """UI 업데이트"""
        # Left gripper - 실제 전류 표시
        if left_curr is not None:
            self.left_load.set(f"{left_curr} mA")
            self.left_load_bar['value'] = min(abs(left_curr), 1000)
            self.left_current.set(f"{left_curr} mA")
            self.left_current_bar['value'] = min(abs(left_curr), 2000)

        # Right gripper - 실제 전류 표시
        if right_curr is not None:
            self.right_load.set(f"{right_curr} mA")
            self.right_load_bar['value'] = min(abs(right_curr), 1000)
            self.right_current.set(f"{right_curr} mA")
            self.right_current_bar['value'] = min(abs(right_curr), 2000)

        # Update position labels
        self.left_pos_label.config(text=str(self.left_position.get()))
        self.right_pos_label.config(text=str(self.right_position.get()))
        self.left_shoulder_label.config(text=str(self.left_shoulder_pos.get()))
        self.right_shoulder_label.config(text=str(self.right_shoulder_pos.get()))

    def record_values(self):
        """현재 값 기록"""
        record = {
            'no': len(self.records) + 1,
            'time': datetime.now().strftime("%H:%M:%S"),
            'j31_pos': self.left_position.get(),
            'j31_curr': self.left_current.get(),
            'j41_pos': self.right_position.get(),
            'j41_curr': self.right_current.get(),
            'memo': self.memo_text.get()
        }

        self.records.append(record)

        # Add to treeview
        self.records_tree.insert('', 'end', values=(
            record['no'], record['time'], record['j31_pos'], record['j31_curr'],
            record['j41_pos'], record['j41_curr'], record['memo']
        ))

        # Clear memo
        self.memo_text.set("")

    def delete_selected(self):
        """선택한 기록 삭제"""
        selected = self.records_tree.selection()
        for item in selected:
            self.records_tree.delete(item)

    def clear_records(self):
        """전체 기록 삭제"""
        if messagebox.askyesno("확인", "모든 기록을 삭제하시겠습니까?"):
            self.records.clear()
            for item in self.records_tree.get_children():
                self.records_tree.delete(item)

    def save_to_txt(self):
        """TXT 파일로 저장"""
        if not self.records:
            messagebox.showwarning("경고", "저장할 기록이 없습니다")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            initialfile=f"gripper_current_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        )

        if not filename:
            return

        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("# 그리퍼 전류 측정 기록\n")
                f.write(f"# 날짜: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("# " + "=" * 50 + "\n\n")

                for record in self.records:
                    f.write(f"[{record['no']}] {record['time']}\n")
                    f.write(f"    J31 (Left):  pos={record['j31_pos']}, current={record['j31_curr']}\n")
                    f.write(f"    J41 (Right): pos={record['j41_pos']}, current={record['j41_curr']}\n")
                    if record['memo']:
                        f.write(f"    메모: {record['memo']}\n")
                    f.write("\n")

            messagebox.showinfo("저장 완료", f"저장됨: {filename}")

        except Exception as e:
            messagebox.showerror("저장 오류", str(e))

    def run(self):
        """메인 루프 실행"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        """종료 시 정리"""
        self.running = False
        if self.ser:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    app = GripperCurrentTool()
    app.run()
