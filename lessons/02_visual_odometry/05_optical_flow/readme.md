# 05. Optical flow — LK와 DIS

04번 실습과 같은 두 장의 영상에서 첫 번째 프레임의 점이 두 번째 프레임에서 어디로 이동했는지 추정한다. 두 예제 모두 오른쪽이 양의 x, 아래쪽이 양의 y이며 변위 단위는 픽셀이다.

| 예제 | 어디에서 변위를 구하는가? | 화면에서 확인할 것 |
|---|---|---|
| 피라미드 Lucas–Kanade (LK) | 첫 영상에서 검출한 코너 중 추적에 성공한 점 | 코너가 어디에 몰려 있는지, 어느 점이 다음 영상까지 추적되는지 |
| Dense Inverse Search (DIS) | 첫 영상의 모든 픽셀 | 물체 경계에서 이동 방향이나 크기가 어떻게 달라지는지 |

## 샘플 실행

이 실습 폴더에서 다음 명령을 실행한다. 각 예제는 창 하나를 열고, 이미지 창에서 키를 누르면 종료된다.

```bash
./run.sh lk
./run.sh dis
```

## LK — 코너 추적

[src/optical_flow.cpp](src/optical_flow.cpp)는 Shi–Tomasi 코너를 최대 300개 검출한 뒤 피라미드 LK로 추적한다. 두 번째 영상에서 첫 번째 영상으로 다시 추적해 원래 코너에서 1픽셀 이내로 돌아온 점을 남긴다. 왼쪽은 검출된 코너, 오른쪽은 이동 화살표와 도착점(초록색)이다.

## DIS — 픽셀별 이동장

[src/dis_optical_flow.cpp](src/dis_optical_flow.cpp)는 OpenCV의 DIS 중간 품질 프리셋으로 픽셀마다 수평·수직 변위를 추정한다. 왼쪽에는 24픽셀 간격으로 화살표를 표시하고 오른쪽에는 전체 이동장을 그린다. 색상은 아래 범례의 방향을, 밝기는 이동 크기를 나타낸다. 0픽셀은 검은색이고 32픽셀 이상은 최대 밝기다. 무늬가 적은 벽에도 값이 나오므로 주변의 뚜렷한 경계와 함께 해석한다. [OpenCV DIS 문서](https://docs.opencv.org/4.x/de/d4f/classcv_1_1DISOpticalFlow.html)

## 같은 입력으로 비교하기

같은 크기의 영상 두 장을 인자로 전달한다. 아래는 이 실습의 `build/`에서 창 없이 실행하는 예다.

```bash
SLAM_HEADLESS=1 ./optical_flow ../data/1.png ../data/2.png
SLAM_HEADLESS=1 ./dis_optical_flow ../data/1.png ../data/2.png
```

각각 `optical_flow.png`와 `dis_optical_flow.png`를 저장한다. 두 실행 파일 모두 터미널에 변위의 성분별 중앙값을 출력한다. LK는 왕복 검사를 통과한 코너에서, DIS는 전체 픽셀에서 계산하므로 두 값은 서로 다를 수 있다. OpenCV 3에서 DIS를 빌드할 때는 contrib의 `optflow` 모듈이 필요하다.
