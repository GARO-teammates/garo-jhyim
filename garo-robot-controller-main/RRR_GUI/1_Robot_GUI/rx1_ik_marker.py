"""
RX-1 IK 엔드포인트 RViz 마커 퍼블리셔
- 현재 FK 위치 (파란 구)
- 목표 IK 위치 (녹색 구)
- 오른팔/왼팔 구분

두 가지 모드로 사용 가능:
1. GUI 내장: GUI와 같은 프로세스 (ROS2 사용 불가 시 무시)
2. 별도 프로세스: rx1_ik_marker_node.py로 실행 (ROS2 Python 환경)
"""

import os
import json
import time
import threading

# IK 마커 데이터 파일 (GUI ↔ 마커 노드 통신용)
MARKER_DATA_FILE = "/tmp/rx1_ik_marker_data.json"


class IKMarkerFileWriter:
    """GUI에서 마커 데이터를 파일로 저장 (ROS2 없이 작동)"""

    def __init__(self):
        self.data = {
            'right': {'fk': None, 'ik': None},
            'left': {'fk': None, 'ik': None},
            'timestamp': 0
        }
        self._write_lock = threading.Lock()

    def update_fk(self, arm, position):
        """FK 위치 업데이트"""
        with self._write_lock:
            self.data[arm]['fk'] = list(position) if position is not None else None
            self._save()

    def update_ik_target(self, arm, position):
        """IK 목표 위치 업데이트"""
        with self._write_lock:
            self.data[arm]['ik'] = list(position) if position is not None else None
            self._save()

    def clear_ik_target(self, arm):
        """IK 목표 마커 제거"""
        with self._write_lock:
            self.data[arm]['ik'] = None
            self._save()

    def _save(self):
        """파일로 저장"""
        try:
            self.data['timestamp'] = time.time()
            with open(MARKER_DATA_FILE, 'w') as f:
                json.dump(self.data, f)
        except Exception:
            pass

    def start(self):
        """시작 (호환성 유지)"""
        return True

    def stop(self):
        """정지 (호환성 유지)"""
        pass


# GUI용 싱글톤
_marker_writer = None


def get_marker_manager():
    """마커 매니저 싱글톤 (GUI용 - 파일 기반)"""
    global _marker_writer
    if _marker_writer is None:
        _marker_writer = IKMarkerFileWriter()
    return _marker_writer


# ============================================================
# 아래는 ROS2 마커 노드 (별도 프로세스로 실행)
# ============================================================

def run_marker_node():
    """ROS2 마커 퍼블리셔 노드 실행 (별도 프로세스용)"""
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    import numpy as np

    class IKMarkerPublisher(Node):
        """IK 엔드포인트 마커 퍼블리셔"""

        def __init__(self):
            super().__init__('ik_marker_publisher')

            self.marker_pub = self.create_publisher(
                MarkerArray, '/ik_markers', 10
            )

            # 주기적으로 파일에서 데이터 읽어서 퍼블리시 (10Hz)
            self.timer = self.create_timer(0.1, self.publish_markers)
            self.get_logger().info('IK Marker Publisher 시작 (파일 모니터링)')

        def publish_markers(self):
            """마커 배열 퍼블리시"""
            # 파일에서 데이터 읽기
            try:
                if not os.path.exists(MARKER_DATA_FILE):
                    return
                with open(MARKER_DATA_FILE, 'r') as f:
                    data = json.load(f)
            except Exception:
                return

            marker_array = MarkerArray()
            marker_id = 0

            for arm in ['right', 'left']:
                arm_offset = 0 if arm == 'right' else 10

                # FK 위치 마커 (파란 구)
                if data.get(arm, {}).get('fk') is not None:
                    fk_pos = data[arm]['fk']
                    fk_marker = self._create_sphere_marker(
                        marker_id + arm_offset, fk_pos,
                        color=(0.2, 0.6, 1.0, 0.8), scale=0.03,
                        ns=f'{arm}_fk'
                    )
                    marker_array.markers.append(fk_marker)
                    marker_id += 1

                # IK 목표 마커 (녹색 구)
                if data.get(arm, {}).get('ik') is not None:
                    ik_pos = data[arm]['ik']
                    ik_marker = self._create_sphere_marker(
                        marker_id + arm_offset, ik_pos,
                        color=(0.2, 1.0, 0.2, 0.9), scale=0.04,
                        ns=f'{arm}_ik_target'
                    )
                    marker_array.markers.append(ik_marker)
                    marker_id += 1

                    # FK→IK 연결선
                    fk_pos = data.get(arm, {}).get('fk')
                    if fk_pos is not None:
                        line_marker = self._create_line_marker(
                            marker_id + arm_offset, fk_pos, ik_pos,
                            color=(1.0, 1.0, 0.0, 0.6),
                            ns=f'{arm}_ik_line'
                        )
                        marker_array.markers.append(line_marker)
                        marker_id += 1

            if marker_array.markers:
                self.marker_pub.publish(marker_array)

        def _create_sphere_marker(self, id, position, color, scale, ns):
            """구 마커 생성"""
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000

            return marker

        def _create_line_marker(self, id, start, end, color, ns):
            """선 마커 생성"""
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            p1 = Point()
            p1.x, p1.y, p1.z = float(start[0]), float(start[1]), float(start[2])
            p2 = Point()
            p2.x, p2.y, p2.z = float(end[0]), float(end[1]), float(end[2])
            marker.points = [p1, p2]

            marker.scale.x = 0.005

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000

            return marker

    # ROS2 노드 실행
    rclpy.init()
    node = IKMarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 테스트 코드
if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == '--ros':
        # ROS2 마커 노드 실행
        print("ROS2 마커 노드 시작...")
        run_marker_node()
    else:
        # GUI 모드 테스트
        print("GUI 모드 테스트 (파일 기반)")
        manager = get_marker_manager()
        manager.start()

        import numpy as np
        for i in range(10):
            t = i * 0.3
            manager.update_fk('right', [
                0.3 + 0.1 * np.cos(t),
                -0.3 + 0.1 * np.sin(t),
                0.2
            ])
            manager.update_ik_target('right', [0.35, -0.25, 0.25])
            print(f"마커 데이터 저장: {MARKER_DATA_FILE}")
            time.sleep(0.3)

        manager.stop()
        print("테스트 완료")
