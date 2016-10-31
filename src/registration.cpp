#include <registration.h>

//setter
void icp_4d::setTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud){
  target_cloud_ = target_cloud;
}

void icp_4d::setSource(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud){
  source_cloud_ = source_cloud;
}

//getter
pcl::PointCloud<pcl::PointXYZI>::Ptr icp_4d::getTarget(){
  return target_cloud_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr icp_4d::getSource(){
  return source_cloud_;
}

void icp_4d::setMaxRange(pcl::PointCloud<pcl::PointXYZI>::Ptr output,
				 pcl::PointCloud<pcl::PointXYZI>::ConstPtr source, 
				 double max_range){

  //create kdtree object
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (source);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  pcl::PointXYZI pt0;
  pt0.x = 0.0;
  pt0.y = 0.0;
  pt0.z = 0.0;
  pt0.intensity = 0.0;

  //apply radius search
  if (kdtree.radiusSearch (pt0, max_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
	//output resize
	output->width = pointIdxRadiusSearch.size ();
	output->height = 1;
	output->resize(output->width * output->height);
	//store pointcloud
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
	  output->points[i].x = source->points[ pointIdxRadiusSearch[i] ].x;
	  output->points[i].y = source->points[ pointIdxRadiusSearch[i] ].y;
	  output->points[i].z = source->points[ pointIdxRadiusSearch[i] ].z;
	  output->points[i].intensity = source->points[ pointIdxRadiusSearch[i] ].intensity;
	}
  }
  else{
	std::cerr << "[max_range_set] no data!" << std::endl;
	exit(-1);
  }

  std::cerr << "range set done ... " << source->size() << " ---> " << output->size() << std::endl;
}


void icp_4d::normalizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output,
						 pcl::PointCloud<pcl::PointXYZI>::ConstPtr source,
						 double max_range,
						 double max_intensity){

  if(source->size() == 0){
	std::cerr << "[normalize] no input data!" << std::endl;
	exit(-1);
  }

  //output resize
  output->width = source->size();
  output->height = 1;
  output->resize(output->width * output->height);

  //normalize
  for(int i = 0; i < output->size(); i++){
	output->points[i].x = source->points[i].x / (2.0 * max_range) + 0.5;
	output->points[i].y = source->points[i].y / (2.0 * max_range) + 0.5;
	output->points[i].z = source->points[i].z / (2.0 * max_range) + 0.5;
	output->points[i].intensity = source->points[i].intensity / max_intensity;
  }

  std::cerr << "normalize done ... " << std::endl;
}


void icp_4d::estimateCorrespond(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						pcl::PointCloud<pcl::PointXYZI>::Ptr source,
						double threshold_dst){

  //clear correspond
  // if(correspond_.size() != 0){
  // 	correspond_.clear();
  // 	correspond_.shrink_to_fit();
  // }

  //create kdtree object
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (target);
  int K = 1; //only nearest neighbor
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for(int i = 0; i < source->size(); i++){
	pcl::PointXYZI searchPoint(source->points[i]);
	kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);


	double corr_dst;
	corr_dst = sqrt( pow( (source->points[i].x - target->points[ pointIdxNKNSearch[0] ].x), 2) +
					 pow( (source->points[i].y - target->points[ pointIdxNKNSearch[0] ].y), 2) +
					 pow( (source->points[i].z - target->points[ pointIdxNKNSearch[0] ].z), 2) );

	if(corr_dst < threshold_dst){
	  //add correspond(source, target)
	  std::pair<int, int> cor(i, pointIdxNKNSearch[0]);
	  correspond_.push_back(cor);
	}
  }

  std::cerr << "estimate correspond done ... corr_size = " << correspond_.size() << std::endl;
}
