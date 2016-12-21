#include <string>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_vis(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_vis(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr init_source(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pre_source(new pcl::PointCloud<pcl::PointXYZI>);
  *init_source = *source_cloud;
  Eigen::Quaternionf v_rotation (0.0, 0.0, 0.0, 0.0);
  Eigen::Vector3f v_translation (50.0, 50.0, 0.0);
  pcl::transformPointCloud(*target_cloud, *target_vis, -v_translation, v_rotation);
  pcl::transformPointCloud(*source_cloud, *source_vis, v_translation, v_rotation);

  //color handler
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity1(target_vis, "intensity");
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity2(source_vis, "intensity");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color1(target_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color3(init_source, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color2(source_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color4(pre_source, 100, 100, 100);

  double threshold_radius = 0.1;
  std::cerr << "[threshold] _ ";
  std::cin >> threshold_radius;

  double intensity_scale = 0.2;
  std::cerr << "[intensity scale] _ ";
  std::cin >> intensity_scale;

  int max_iter = 2;
  std::cerr << "[loop] _ ";
  std::cin >> max_iter;

  //create viewer object
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  int v1(0), v2(0);//viewport
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setCameraPosition (0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0);
  viewer->setCameraClipDistances (0.0, 1.0);
  
  viewer->addPointCloud<pcl::PointXYZI> (target_vis, intensity1, "target_v", v1);
  viewer->addPointCloud<pcl::PointXYZI> (source_vis, intensity2, "source_v", v1);
  viewer->addPointCloud<pcl::PointXYZI> (target_cloud, color1, "target", v2);
  viewer->addPointCloud<pcl::PointXYZI> (init_source, color3, "source_i", v2);

  int converged = -1;  
  for(int i = 0; i < max_iter; i++){
	std::cerr << "loop [" << i << "]" << std::endl;
	test.estimateCorrespond_4d(target_cloud, source_cloud, intensity_scale, threshold_radius);
	test.estimateTransform(target_cloud, source_cloud);
	Eigen::Matrix4f result = test.getTransformation();
	//transform
	*pre_source = *source_cloud;
	std::cerr << " pre_size : " << pre_source->points.size() << std::endl;
	pcl::transformPointCloud(*source_cloud, *source_cloud, result);
	std::cerr << " after_size : " << source_cloud->points.size() << std::endl;
	//update visualizer
	if( !viewer->updatePointCloud (source_cloud, color2, "source_m") )
	  viewer->addPointCloud<pcl::PointXYZI> (source_cloud, color2, "source_m", v2);
	if( !viewer->updatePointCloud (pre_source, color4, "source_pre") )
	  viewer->addPointCloud<pcl::PointXYZI> (source_cloud, color4, "source_pre", v2);
	//viewer->spinOnce(100);
	viewer->spinOnce(1);	

	//loop check
	if(test.getEpsilon() < 0.0001){
	  converged = i;
	  std::cerr << "converge pattern 1" << std::endl;
	  break;
	}
	// if(test.getNumberOfNewCorrespond() == 0){
	//   converged = i;
	//   std::cerr << "converge pattern 2" << std::endl;
	//   break;
	// }
  }
  
  if(converged > 0){
	std::cerr << "ICP is converged. loop : " << converged << std::endl;
  }
  else{
	std::cerr << "ICP is not converged. loop : " << max_iter << std::endl;
  }

  //show final correspondence
  std::vector< std::pair<int, int> > corresponds = test.getCorrespond();
  int counter = 0;
  for(int i = 0; i < corresponds.size(); i++){
	if(counter > 1000)
	  break;
	// Get the pair of points
	auto p_left = target_vis->points[corresponds.at(i).second];
	auto p_right = source_vis->points[corresponds.at(i).first];
	
	// Generate a random (bright) color
	double r = (rand() % 100);
	double g = (rand() % 100);
	double b = (rand() % 100);
	double max_channel = std::max(r, std::max(g, b));
	r /= max_channel;
	g /= max_channel;
	b /= max_channel;

	// Draw the line
	std::stringstream l_id;
	l_id << "line_" << i ;
	viewer->addLine(p_left, p_right, r, g, b, l_id.str(), v1);
	counter++;
  }
  viewer->spin();  
  return 0;
}



