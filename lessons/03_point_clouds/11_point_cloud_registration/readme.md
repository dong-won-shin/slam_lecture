# 11. 점군 정합

대응점 기반 SVD 정합과 반복 최근접점 정합(ICP)을 실험한다. RGB-D 영상에서 구한 강체 변환과 방 스캔 두 장의 ICP 수렴 여부를 각각 확인합니다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./closed_form_registration ../data/1.png ../data/2.png ../data/1_depth.png ../data/2_depth.png
./iterative_form_registration ../data/room_scan1.pcd ../data/room_scan2.pcd
```
