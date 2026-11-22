# 11. 점군 정합

대응점 기반 SVD 정합과 반복 최근접점 정합(ICP)을 실험한다. RGB-D 영상에서 구한 강체 변환과 방 스캔 두 장의 ICP 수렴 여부를 각각 확인합니다.

![두 실내 스캔의 같은 높이 구간을 XY 평면에 투영한 ICP 변환 전후 비교](figures/icp-room.png)

주황색은 고정한 `room_scan2`, 청록색은 움직이는 `room_scan1`이다. 오른쪽은 실행 파일이 출력한 4×4 변환 행렬을 첫 번째 스캔에 적용한 결과다. 두 그림 모두 $|z|<0.35$ m인 점을 위에서 본다. 왼쪽 벽의 방향은 조금 바뀌지만 겹치지 않는 윤곽도 남는다. 터미널의 `has converged:1`은 ICP가 종료 조건에 도달했다는 뜻이므로 실제 정합 상태도 함께 확인한다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./closed_form_registration ../data/1.png ../data/2.png ../data/1_depth.png ../data/2_depth.png
./iterative_form_registration ../data/room_scan1.pcd ../data/room_scan2.pcd
```
