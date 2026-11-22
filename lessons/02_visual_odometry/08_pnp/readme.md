# 08. PnP와 카메라 자세

깊이 영상으로 얻은 3D 점과 다음 영상의 2D 점으로 카메라 자세를 구한다. OpenCV PnP로 구한 자세와 Gauss–Newton 및 g2o로 구한 자세를 비교합니다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./pose_estimation_3d2d ../data/1.png ../data/2.png ../data/1_depth.png ../data/2_depth.png
```
