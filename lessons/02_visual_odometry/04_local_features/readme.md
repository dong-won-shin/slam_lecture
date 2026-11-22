# 04. 지역 특징점과 매칭

같은 영상 쌍에서 ORB와 SuperPoint로 대응점을 찾는다. 전체 매칭에서 어떤 대응점이 필터링 후에도 남는지 두 영상을 함께 보며 확인합니다.

![ORB로 찾은 특징점과 해밍 거리 필터 전후의 대응선](figures/orb-matches.jpg)

아래 오른쪽에는 ORB가 찾은 대응 500쌍이, 위에는 해밍 거리 기준을 통과한 79쌍이 보인다. 위쪽의 선 하나를 양 끝까지 따라가며 모니터 모서리처럼 같은 물체의 같은 지점을 잇는지 확인한다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./orb_cv ../data/1.png ../data/2.png
./orb_self ../data/1.png ../data/2.png
```

## SuperPoint로 두 영상 매칭하기

이 실습의 `build/`에서 아래 명령을 실행합니다. 저장소의 `data/1.png`와 `data/2.png`가 기본 입력입니다.

```bash
./superpoint-env/bin/python ../src/superpoint.py --show
```

![같은 책상 장면에서 SuperPoint로 검출한 특징점과 RANSAC 검증 후의 대응](figures/superpoint.jpg)

결과는 `superpoint_matches.png`에 저장됩니다. 창의 위쪽은 두 영상에서 검출한 특징점입니다. 아래쪽은 256차원 디스크립터의 L2 거리로 상호 최근접 매칭을 구한 뒤, 기본 행렬(fundamental matrix)을 RANSAC으로 추정해 남긴 대응점입니다. 거리가 짧은 순으로 최대 50쌍을 표시합니다. 키를 누르면 창이 닫힙니다.

다른 영상은 경로 두 개를 인자로 전달하세요. `--show`를 생략하면 PNG만 저장합니다.

```bash
./superpoint-env/bin/python ../src/superpoint.py ../data/1.png ../data/2.png --output result.png
```

### 새 Mac에 예제 설치

저장소 루트에서 `./build_3rdparty.sh`를 실행한 뒤 다음 명령으로 별도 Python 환경을 만듭니다.

```bash
cd lessons/02_visual_odometry/04_local_features
../../../build/tools/bin/micromamba --no-rc create -y --root-prefix "$PWD/../../../build/mamba" \
  --prefix "$PWD/build/superpoint-env" --override-channels -c conda-forge python=3.11 pip
./build/superpoint-env/bin/python -m pip install -r requirements-superpoint.txt
```

첫 실행 시 사전학습 가중치를 `build/model-cache/`에 내려받습니다. CPU에서 추론합니다. 모델의 출처와 사용 조건은 [SuperPoint 모델 카드](https://huggingface.co/magic-leap-community/superpoint)를 참고하세요.

## 실행 영상

![ORB 매칭 뷰어에서 책상 장면의 대응선을 확대하는 모습](media/preview.gif)

ORB 뷰어에서 상단의 대응선을 확대했다가 원래 배율로 돌아온다. 선의 양 끝이 두 영상에서 같은 물체의 같은 모서리에 놓이는지 살펴본다. [MP4로 보기 · 7.6초](media/demo.mp4)
