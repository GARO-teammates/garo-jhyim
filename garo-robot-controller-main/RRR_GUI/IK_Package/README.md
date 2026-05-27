# IK_Package - 역기구학 솔버

## 파일
- `rx1_ik_custom.py`: RX1 로봇용 커스텀 IK 솔버
- `example_usage.py`: 사용 예제
- `install.sh`: 설치 스크립트

## 사용법
```python
from rx1_ik_custom import RX1IKSolver

solver = RX1IKSolver()
angles = solver.solve_ik(target_position, target_orientation)
```
