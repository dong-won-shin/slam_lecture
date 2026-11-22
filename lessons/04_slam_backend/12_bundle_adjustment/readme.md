# 12. 번들 조정

여러 영상의 재투영 오차를 줄이도록 카메라와 3D 점을 함께 최적화한다. Ceres의 초기·최종 비용과 최적화 전후 점군인 `initial.ply`, `final.ply`를 비교합니다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./bundle_adjustment_ceres ../data/problem-16-22106-pre.txt
```
