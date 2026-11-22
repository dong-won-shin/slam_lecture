# SLAM 강의 예제

## macOS에서 빌드

저장소 루트에서 실행합니다. Xcode Command Line Tools가 필요합니다. `xcode-select -p`에서 경로를 찾지 못하면 `xcode-select --install`로 설치한 뒤 진행하세요.

```bash
./build_3rdparty.sh
./build.sh
```

첫 명령은 micromamba로 CMake, Ninja, Eigen, OpenCV, PCL, Ceres를 `build/deps/`에 설치하고 OpenGV 소스를 준비합니다. 처음 한 번은 인터넷 연결이 필요합니다. Homebrew나 관리자 권한은 필요하지 않습니다. 두 번째 명령은 저장소에 포함된 g2o, Sophus, DBoW3와 함께 강의 예제 23개를 빌드합니다. 소스만 바꾼 뒤에는 `./build.sh`만 다시 실행하면 됩니다.

실행 파일은 각 강의 소스 폴더의 `build/`에 생성됩니다. 공용 라이브러리와 컴파일 캐시는 저장소 루트의 `build/`에서 관리합니다. 예를 들어 저장소 루트에서 다음과 같이 번들 조정 예제를 빌드하고 실행할 수 있습니다.

```bash
cd BundleAdjustment/build
cmake ..
ninja
./bundle_adjustment_ceres ../data/problem-16-22106-pre.txt
```

각 강의의 `build/`에서 `cmake ..`로 빌드를 구성하고 `ninja`로 해당 강의와 필요한 라이브러리를 빌드할 수 있습니다. 다음 빌드부터는 `ninja`만 실행하면 됩니다. 두 명령이 PATH에 없으면 같은 위치에서 `../../build/deps/bin/cmake ..`와 `../../build/deps/bin/ninja`를 사용하세요. 위 예제는 현재 폴더에 `initial.ply`와 `final.ply`를 저장합니다.

## 전체 예제 실행

```bash
./build/deps/bin/ctest --test-dir build/macos --output-on-failure -j 4
```

CTest는 저장소에 포함된 이미지, 포인트 클라우드, 그래프, 번들 조정 데이터를 사용합니다. 자체 입력을 생성하는 예제는 인자 없이 실행합니다. 시각화 예제는 자동 실행 시 창을 띄우지 않고 결과를 저장합니다. 생성된 PNG, PCD, PLY, 그래프와 어휘 파일은 `build/macos/results/<예제 이름>/`에서 확인할 수 있습니다.

## 개별 예제 실행

아래 명령은 저장소 루트에서 실행합니다. 이미지 창에서 키를 누르면 종료됩니다.

```bash
./KalmanFilter/build/kalman-test
'./Keypoint&Descriptor/build/orb_cv' 'Keypoint&Descriptor/data/1.png' 'Keypoint&Descriptor/data/2.png'
./Triangulation/build/triangulation Triangulation/data/1.png Triangulation/data/2.png
./3DPointCloudProcessing/build/normal_estimation 3DPointCloudProcessing/data/ne_data.pcd
```

`normal_estimation`은 현재 디렉터리에 `p_n_cloud_c.pcd`를 쓰고 3D 뷰어를 엽니다. 다른 예제의 실행 파일과 입력 인자는 아래 명령으로 확인할 수 있습니다.

```bash
./build/deps/bin/ctest --test-dir build/macos -N -V
```

## Jetson / Ubuntu

```bash
./build_3rdparty.sh
./build.sh
```

각 예제 디렉터리의 `build/` 안에서 실행합니다.
