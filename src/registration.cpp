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
								double threshold_dst,
								double max_range){
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

	if( corr_dst < threshold_dst / (2*max_range) ){
	  //add correspond(source, target)
	  std::pair<int, int> cor(i, pointIdxNKNSearch[0]);
	  correspond_.push_back(cor);
	}
  }

  std::cerr << "estimate correspond done ... corr_size = " << correspond_.size() << std::endl;
}


double icp_4d::estimateEpsilon(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
							   pcl::PointCloud<pcl::PointXYZI>::Ptr source){
  if(correspond_.size() == 0){
	std::cerr << "[estimate epsilon] no correspond data!" << std::endl;
	return -1;
  }

  double epsilon = 0.0;
  for(int i = 0; i < correspond_.size(); i++){
	double corr_dst = 0.0;
	corr_dst = sqrt( pow( (source->points[ correspond_.at(i).first ].x - target->points[ correspond_.at(i).second ].x), 2) +
					 pow( (source->points[ correspond_.at(i).first ].y - target->points[ correspond_.at(i).second ].y), 2) +
					 pow( (source->points[ correspond_.at(i).first ].z - target->points[ correspond_.at(i).second ].z), 2) );
	epsilon += corr_dst;
  }
  epsilon /= correspond_.size();
  std::cerr << "estimate epsilon done ... epsilon=" << epsilon << std::endl;
  return epsilon;
}


void icp_4d::estimateTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
							   pcl::PointCloud<pcl::PointXYZI>::Ptr source){
  if(correspond_.size() == 0){
	std::cerr << "[estimate epsilon] no correspond data!" << std::endl;
	exit(-1);
  }
  std::cerr << "correspond size: " << correspond_.size() << std::endl;

  //estimate mean
  Eigen::Vector3f source_mean(0.0, 0.0, 0.0);
  Eigen::Vector3f target_mean(0.0, 0.0, 0.0);
  for(int i = 0; i < correspond_.size(); i++){
	//sampling
	Eigen::Vector3f source_sample( source->points[ correspond_.at(i).first ].x,
								   source->points[ correspond_.at(i).first ].y,
								   source->points[ correspond_.at(i).first ].z);
	Eigen::Vector3f target_sample( target->points[ correspond_.at(i).second ].x,
								   target->points[ correspond_.at(i).second ].y,
								   target->points[ correspond_.at(i).second ].z);
	//add
	source_mean = source_mean + source_sample;
	target_mean = target_mean + target_sample;
  }
  //devide
  source_mean = source_mean.array() / correspond_.size();
  target_mean = target_mean.array() / correspond_.size();
  std::cerr << "source mean : " << std::endl << source_mean << std::endl;
  std::cerr << "target mean : " << std::endl << target_mean << std::endl;

  Eigen::Matrix3f mat_h = Eigen::Matrix3f::Identity();
  double Sxx = 0.0, Sxy = 0.0, Sxz = 0.0;
  double Syx = 0.0, Syy = 0.0, Syz = 0.0;
  double Szx = 0.0, Szy = 0.0, Szz = 0.0;
  for(int i = 0; i < correspond_.size(); i++){
	//sampling
	Eigen::Vector3f source_subtracted
	  ( source->points[ correspond_.at(i).first ].x - source_mean(0),
		source->points[ correspond_.at(i).first ].y - source_mean(1),
		source->points[ correspond_.at(i).first ].z - source_mean(2));

	Eigen::Vector3f target_subtracted
	  ( target->points[ correspond_.at(i).second ].x - target_mean(0),
		target->points[ correspond_.at(i).second ].y - target_mean(1),
		target->points[ correspond_.at(i).second ].z - target_mean(2));

	//estimate component
	double sxx_ = target_subtracted(0) * source_subtracted(0); Sxx += sxx_;
	double sxy_ = target_subtracted(0) * source_subtracted(1); Sxy += sxy_;
	double sxz_ = target_subtracted(0) * source_subtracted(2); Sxz += sxz_;
	double syx_ = target_subtracted(1) * source_subtracted(0); Syx += syx_;
	double syy_ = target_subtracted(1) * source_subtracted(1); Syy += syy_;
	double syz_ = target_subtracted(1) * source_subtracted(2); Syz += syz_;
	double szx_ = target_subtracted(2) * source_subtracted(0); Szx += szx_;
	double szy_ = target_subtracted(2) * source_subtracted(1); Szy += szy_;
	double szz_ = target_subtracted(2) * source_subtracted(2); Szz += szz_;
  }
  //devide
  Sxx /= correspond_.size(); Sxy /= correspond_.size(); Sxz /= correspond_.size();
  Syx /= correspond_.size(); Syy /= correspond_.size(); Syz /= correspond_.size();
  Szx /= correspond_.size(); Szy /= correspond_.size(); Szz /= correspond_.size();

  mat_h << Sxx, Sxy, Sxz,
	       Syx, Syy, Syz,
	       Szx, Szy, Szz;

  std::cerr << "mat H:" << std::endl
			<< mat_h << std::endl;

  //apply SVD
  Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> > svd(mat_h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::MatrixUType& u = svd.matrixU();
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::MatrixVType& v = svd.matrixV();
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::SingularValuesType& sv = svd.singularValues();
  std::cerr << "singular values" << std::endl
			<< sv << std::endl;
  std::cerr << "matrix V" << std::endl << v << std::endl;
  std::cerr << "matrix U" << std::endl << u << std::endl;
  Eigen::Matrix3f ut = u.transpose();
  rotation_guess_ = v * ut;
  translate_guess_ = target_mean - rotation_guess_ * source_mean;

  std::cerr << "rotation:" << std::endl
			<< rotation_guess_.matrix() << std::endl;
  std::cerr << "translation: " << std::endl
			<< translate_guess_ << std::endl;

}



void icp_4d::applyICP(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
					  pcl::PointCloud<pcl::PointXYZI>::Ptr source,
					  double max_range,
					  double max_intensity,
					  double threshold_dst){







}
