# 14. 차량 주행 궤적의 포즈 그래프 최적화

KITTI Odometry 시퀀스 00에서 차량이 약 3.63km를 주행하고 출발 지점으로 돌아오는 구간을 사용한다. 왼쪽은 방향 오차가 누적된 초기 궤적이고, 오른쪽은 네 곳의 재방문 제약을 적용한 g2o 최적화 결과다.

저장소 루트에서 실행한다. 주행 그래프는 실습에 포함되어 있다.

```bash
./lessons/04_slam_backend/14_pose_graph/run.sh
```

두 패널에서 **노란 고리는 출발점**, **흰 십자는 마지막 위치**다. `Start to end`는 이 두 자세 사이의 3D 직선거리다. 초기에는 96.27m였던 간격이 최적화 후 0.36m로 줄어든다. KITTI 기준 궤적의 같은 간격은 0.36m다. 청록색 궤적은 연속된 자세를 잇고, 분홍색 고리와 선은 같은 장소를 다시 방문한 두 시점을 연결한다.

| 조작 | 기능 |
|---|---|
| 어느 쪽에서든 드래그, `+` / `-` | Y축을 위로 두고 두 궤적을 함께 회전, 확대 / 축소 |
| `0`을 누른 뒤 `n` / `p` | 오른쪽에서 실제 최적화 반복을 앞으로 / 뒤로 한 단계씩 확인 |
| `e` | 최종 궤적으로 이동 |
| `c` | 재방문 제약을 숨기거나 다시 표시 |
| `r` / `q` 또는 `Esc` | 시점 초기화 / 종료 |

실행하면 최적화 결과를 실습의 `build/result.g2o`에 저장한다. 창 없이 실행하면 초기·최종 궤적 비교 화면도 `build/pose_graph.png`에 저장한다. 기존 구면 그래프는 `sphere` 옵션으로 열 수 있다.

```bash
SLAM_HEADLESS=1 ./lessons/04_slam_backend/14_pose_graph/run.sh
./lessons/04_slam_backend/14_pose_graph/run.sh sphere
```

## 그래프를 구성한 방법

기준 궤적의 15~4464번 프레임에서 매 10번째 프레임과 재방문 프레임을 선택해 451개 자세를 구성했다. 시작 자세를 고정하고 KITTI 카메라 좌표의 Y·Z 부호를 함께 바꿔 오른손 Y-up 좌표계로 표현했다.

- **오도메트리 제약 450개**는 연속된 키프레임의 기준 상대 자세에 이동거리 1m당 0.007도의 요(yaw) 편향을 넣어 만들었다. 이 상대 자세를 차례로 누적한 결과가 왼쪽 궤적이다.
- **재방문 제약 4개**는 13번 실습에서 영상으로 검증한 원본 프레임 쌍 505↔3499, 732↔3670, 920↔3830, 15↔4464를 사용한다. 두 자세 사이의 이동과 회전은 KITTI 기준 궤적에서 계산했다.
- 두 종류의 제약을 함께 최적화한다. 화면의 `Chi-squared`는 상대 자세 오차에 각 제약의 정보 행렬을 적용한 가중 제곱합이다. 출발·도착 간격은 마지막 재방문 제약이 얼마나 잘 맞는지 보여준다.

그래프를 다시 만들려면 KITTI의 odometry ground truth에서 `poses/00.txt`를 꺼내 다음 명령을 실습 폴더에서 실행한다. 생성 스크립트는 Python, NumPy, OpenCV를 사용한다.

```bash
python3 data/make_kitti_graph.py /path/to/poses/00.txt data/kitti_00.g2o
```

`data/kitti_00.g2o`는 Andreas Geiger, Philip Lenz, Christoph Stiller, Raquel Urtasun의 KITTI 데이터를 위 방식으로 가공한 자료이며 원본과 같은 CC BY-NC-SA 3.0으로 제공한다. [KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) · [KITTI 이용 조건](https://www.cvlibs.net/datasets/kitti/) · [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/)
