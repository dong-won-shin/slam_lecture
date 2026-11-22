# 14. 포즈 그래프 최적화

상대 자세 제약을 이용해 그래프 전체의 카메라 자세를 조정한다. 반복별 오차를 확인하고 입력 `sphere.g2o`와 출력 `result.g2o`의 정점 자세를 비교합니다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./pose_graph_g2o_SE3 ../data/sphere.g2o
```
