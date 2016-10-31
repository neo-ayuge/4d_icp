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

  //pretreatment
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_target (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_source (new pcl::PointCloud<pcl::PointXYZI>);
  double max_range = 20.0; double max_intensity = 255.0;

  test.setMaxRange(circle_target, target_cloud, max_range);
  test.setMaxRange(circle_source, source_cloud, max_range);
  //pcl::io::savePCDFile("circle.pcd", *circle_target);

  pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_target (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_source (new pcl::PointCloud<pcl::PointXYZI>);
  test.normalizePointCloud(normalized_target, circle_target, max_range, max_intensity);
  test.normalizePointCloud(normalized_source, circle_source, max_range, max_intensity);
  //pcl::io::savePCDFile("normalized.pcd", *normalized_target);

  double threshold_dst = 0.0001;
  test.estimateCorrespond(normalized_target, normalized_source, threshold_dst);

  return 0;

}
