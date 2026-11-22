#include <cmath>
#include <cstdlib>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkRendererCollection.h>

static void keep_y_up(vtkObject *caller, unsigned long, void *, void *) {
  auto *camera = vtkRenderer::SafeDownCast(caller)->GetActiveCamera();
  const double *dir = camera->GetDirectionOfProjection();
  const double horizontal = std::hypot(dir[0], dir[2]);
  // At either pole world Y would project to a point. Keep a small pitch margin.
  const double min_horizontal = 0.02;
  if (horizontal < min_horizontal) {
    const double azimuth = std::atan2(dir[0], dir[2]);
    const double dy = std::copysign(std::sqrt(1 - min_horizontal * min_horizontal), dir[1]);
    const double *focus = camera->GetFocalPoint();
    const double distance = camera->GetDistance();
    camera->SetPosition(focus[0] - distance * min_horizontal * std::sin(azimuth),
                        focus[1] - distance * dy,
                        focus[2] - distance * min_horizontal * std::cos(azimuth));
  }
  camera->SetViewUp(0, 1, 0);
}

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // 입력 포인트 클라우드 저장할
                                           // 오브젝트
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>); // 계산된 Normal을 저장할 오브젝트
  pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

  pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);
  std::cout << "INPUT " << cloud->points.size() << std::endl;
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);   // Use all neighbors in a sphere of radius 3cm
                              // setKSearch() 변경 가능
  ne.compute(*cloud_normals); // Compute the features

  // 포인트수 출력
  std::cout << "NORMAL " << cloud_normals->points.size() << std::endl;

  // Copy the point cloud data
  pcl::concatenateFields(*cloud, *cloud_normals, p_n_cloud_c);
  pcl::io::savePCDFile<pcl::PointNormal>("p_n_cloud_c.pcd", p_n_cloud_c);

  std::cerr << "Cloud C: " << std::endl;
  for (std::size_t i = 0; i < 5; ++i)
    std::cerr << "    " << p_n_cloud_c[i].x << " " << p_n_cloud_c[i].y << " "
              << p_n_cloud_c[i].z << " " << p_n_cloud_c[i].normal[0] << " "
              << p_n_cloud_c[i].normal[1] << " " << p_n_cloud_c[i].normal[2]
              << std::endl;

  // Visualize them.
  if (std::getenv("SLAM_HEADLESS")) return 0;
  pcl::visualization::PCLVisualizer viewer("Normals");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  // Display one normal out of 20, as a line of length 3cm.
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals,
                                                           20, 0.03, "normals");
  // Apply the same up direction after trackball gestures and PCL camera resets.
  vtkNew<vtkCallbackCommand> y_up;
  y_up->SetCallback(keep_y_up);
  auto *renderer = viewer.getRendererCollection()->GetFirstRenderer();
  renderer->AddObserver(vtkCommand::StartEvent, y_up);
  viewer.spin();
}
