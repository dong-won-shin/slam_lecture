# 05. Optical flow

04번에서 매칭에 사용한 두 영상으로 피라미드 Lucas–Kanade 추적을 실험한다. 아래 명령은 이 실습의 `build/`에서 실행한다.

```bash
./optical_flow ../data/1.png ../data/2.png
```

첫 영상에서 Shi–Tomasi 코너를 최대 300개 검출한다. 피라미드 Lucas–Kanade(LK)로 두 번째 영상의 위치를 찾고, 다시 첫 영상으로 추적했을 때 원래 점에서 1픽셀 이내로 돌아온 대응점만 표시한다.

왼쪽은 검출된 코너, 오른쪽은 첫 위치에서 두 번째 위치로 향하는 화살표다. 초록색 점이 도착 위치다. 터미널에는 유효한 추적 수와 수평·수직 변위 중앙값이 픽셀 단위로 출력된다. 양의 x는 오른쪽, 양의 y는 아래쪽이다. `SLAM_HEADLESS=1 ./optical_flow ../data/1.png ../data/2.png`로 실행하면 현재 폴더에 `optical_flow.png`가 저장된다.
