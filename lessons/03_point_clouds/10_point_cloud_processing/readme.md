# 10. 포인트 클라우드 처리

점군을 필터링하고 법선을 추정한 뒤 군집을 분리한다. 필터 전후 점 개수, 법선 방향, 군집별로 저장되는 PCD 파일을 확인합니다.

아래 명령은 이 실습의 `build/`에서 실행합니다.

```bash
./voxel_grid_filter ../data/vgf_data.pcd
./statistical_outlier_filter ../data/sof_data.pcd
./pass_through_filter ../data/ptf_data.pcd
./normal_estimation ../data/ne_data.pcd
./clustering ../data/clustering_data.pcd
```
