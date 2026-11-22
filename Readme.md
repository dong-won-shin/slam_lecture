# SLAM 강의 실습

한 장의 영상에 3D 점을 투영하는 것부터 시작해 두 시점 사이의 운동을 추정하고, 마지막에는 여러 관측의 오차를 함께 줄인다. 14개 실습은 각각 독립적으로 실행할 수 있다. 04~09번은 특징점에서 카메라 자세까지 이어지는 비주얼 오도메트리 순서로, 12~14번은 번들 조정·장소 인식·포즈 그래프 최적화 순서로 구성했다.

## 강의 순서

| 실습 | 단원 | 주제와 실행 방법 | 실습 목표 |
|---|---|---|---|
| 01 | 기초 | [센서 좌표계와 카메라 투영](lessons/01_foundations/01_camera_projection/readme.md) | LiDAR 점을 카메라 좌표계로 옮긴 뒤 영상에 투영한다. |
| 02 | 기초 | [비선형 최소제곱](lessons/01_foundations/02_least_squares/readme.md) | 같은 곡선 피팅 문제를 Gauss–Newton, Ceres, g2o로 푼다. |
| 03 | 기초 | [칼만 필터](lessons/01_foundations/03_kalman_filter/readme.md) | 위치 관측값으로 위치·속도·가속도를 추정한다. |
| 04 | 비주얼 오도메트리 | [지역 특징점과 매칭](lessons/02_visual_odometry/04_local_features/readme.md) | 같은 영상 쌍에서 ORB와 SuperPoint로 대응점을 찾는다. |
| 05 | 비주얼 오도메트리 | [Optical flow](lessons/02_visual_odometry/05_optical_flow/readme.md) | 실내 촬영 동영상의 연속 프레임에서 LK의 코너 추적과 DIS의 픽셀별 이동장을 비교한다. |
| 06 | 비주얼 오도메트리 | [에피폴라 기하](lessons/02_visual_odometry/06_epipolar_geometry/readme.md) | 2D 대응점으로 기본 행렬과 본질 행렬을 구하고 상대 자세를 복원한다. |
| 07 | 비주얼 오도메트리 | [두 시점 삼각측량](lessons/02_visual_odometry/07_triangulation/readme.md) | 두 카메라의 자세와 대응점으로 3D 점을 복원한다. |
| 08 | 비주얼 오도메트리 | [OpenGV 삼각측량](lessons/02_visual_odometry/08_opengv_triangulation/readme.md) | 정답 3D 좌표가 있는 합성 장면에서 두 삼각측량 방법을 비교한다. |
| 09 | 비주얼 오도메트리 | [PnP와 카메라 자세](lessons/02_visual_odometry/09_pnp/readme.md) | 깊이 영상으로 얻은 3D 점과 다음 영상의 2D 점으로 카메라 자세를 구한다. |
| 10 | 포인트 클라우드 | [포인트 클라우드 처리](lessons/03_point_clouds/10_point_cloud_processing/readme.md) | 점군을 필터링하고 법선을 추정한 뒤 군집을 분리한다. |
| 11 | 포인트 클라우드 | [점군 정합](lessons/03_point_clouds/11_point_cloud_registration/readme.md) | 대응점 기반 SVD 정합과 반복 최근접점 정합(ICP)을 실험한다. |
| 12 | SLAM 백엔드 | [번들 조정](lessons/04_slam_backend/12_bundle_adjustment/readme.md) | 여러 영상의 재투영 오차를 줄이도록 카메라와 3D 점을 함께 최적화한다. |
| 13 | SLAM 백엔드 | [루프 폐쇄 후보 검색](lessons/04_slam_backend/13_loop_closure/readme.md) | 시각 단어 사전으로 과거 영상 중 같은 장소의 후보를 검색한다. |
| 14 | SLAM 백엔드 | [포즈 그래프 최적화](lessons/04_slam_backend/14_pose_graph/readme.md) | 상대 자세 제약을 이용해 그래프 전체의 카메라 자세를 조정한다. |

## 폴더 구조

```text
lessons/
  01_foundations/       좌표계, 최적화, 상태 추정
  02_visual_odometry/   영상 특징점, 다중 시점 기하, 자세 추정
  03_point_clouds/      점군 처리와 정합
  04_slam_backend/      번들 조정, 루프 폐쇄 후보 검색, 포즈 그래프
common/                 예제 간에 공유하는 시각화 코드
3rdparty/               저장소에 포함된 외부 라이브러리
cmake/                  공용 빌드 설정과 CTest 정의
scripts/                환경 설치 스크립트
```

각 실습의 `src/`에는 코드가 있고, 파일 입력이 필요한 실습에는 `data/`가 있다. 실행 파일은 해당 실습의 `build/`에 생성된다. 저장소 루트의 `build/`는 공용 의존성과 컴파일 캐시에 사용한다.

## macOS에서 시작하기

Xcode Command Line Tools가 필요하다. `xcode-select -p`에서 경로를 찾을 수 없으면 `xcode-select --install`로 설치한다. 아래 명령은 저장소 루트에서 실행한다.

```bash
./build_3rdparty.sh
./build.sh
./lessons/01_foundations/01_camera_projection/run.sh
```

첫 명령은 micromamba로 CMake, Ninja, Eigen, OpenCV, PCL, Ceres를 설치하고 OpenGV를 준비한다. 처음 설치할 때는 인터넷 연결이 필요하다. 두 번째 명령은 C++ 예제 25개를 빌드한다. Python으로 실행하는 [SuperPoint 실습](lessons/02_visual_odometry/04_local_features/readme.md#superpoint로-두-영상-매칭하기)은 04번 실습 안내에서 별도 환경을 설치한다.

## 개별 실습 다시 빌드하기

각 실습의 `run.sh`는 샘플 데이터로 대표 예제를 실행한다. 실행 파일이나 입력을 직접 선택하려면 해당 실습의 `build/`에서 실행한다. 예를 들어 번들 조정은 다음과 같다.

```bash
cd lessons/04_slam_backend/12_bundle_adjustment/build
cmake ..
ninja
./bundle_adjustment_ceres ../data/problem-16-22106-pre.txt
```

이 폴더에 최적화 전후의 `initial.ply`와 `final.ply`가 저장된다. CMake와 Ninja가 셸의 PATH에 없으면 같은 위치에서 각각 `../../../../build/deps/bin/cmake ..`와 `../../../../build/deps/bin/ninja`를 사용할 수 있다.

## 전체 예제 검증

```bash
./build/deps/bin/ctest --test-dir build/macos --output-on-failure -j 4
```

전체 테스트는 화면을 열지 않고 샘플 데이터로 실행된다. PNG, PCD, PLY, 그래프 등의 출력은 `build/macos/results/<실행 파일 이름>/`에 저장된다. 한 예제만 자세히 보려면 예를 들어 `-R '^orb_cv$' -V`를 추가한다.

## Jetson / Ubuntu

저장소 루트에서 `./build_3rdparty.sh`와 `./build.sh`를 차례로 실행한다. 08번 OpenGV 실습은 Linux 기본 빌드에서 제외되며, 이 실습을 개별 빌드하려면 OpenGV 설치가 필요하다.
