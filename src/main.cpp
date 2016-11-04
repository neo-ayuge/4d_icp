#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <registration.h>

int main(int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if(argc != 3){
	std::cerr << "error" << std::endl;
	return -1;
  }

  pcl::io::loadPCDFile(argv[1], *target_cloud); 
  pcl::io::loadPCDFile(argv[2], *source_cloud); 

  icp_4d test;
  test.setTarget(target_cloud);
  test.setSource(source_cloud);

  Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  
  double max_range = 100.0;
  double max_intensity = 255.0;
  double threshold_radius = 0.2;

  test.applyICP(max_range, max_intensity, threshold_radius, init_guess);

  //test.estimateCorrespond(normalized_target, normalized_source, threshold_dst, max_range);
  //test.estimateEpsilon(normalized_target, normalized_source);
  //test.estimateTransform(normalized_target, normalized_source);

  return 0;

}
