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

std::vector< std::pair<int, int> > icp_4d::getCorrespond(){
  return correspond_;
}

Eigen::Matrix4f icp_4d::getTransformation(){
  return transform_guess_;
}

double icp_4d::getEpsilon(){
  return error_value_;
}

int icp_4d::getNumberOfNewCorrespond(){
  return num_of_new_point_;
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

  //create kdtree object
  if(kdtree_.getInputCloud () == 0){
	kdtree_.setInputCloud (target);
	std::cerr << "kdtree_stored(1)" << std::endl;
  }

  //new_correspond
  std::vector< std::pair<int, int> > new_correspond;

  int K = 1; //only nearest neighbor
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for(int i = 0; i < source->size(); i++){
	pcl::PointXYZI searchPoint(source->points[i]);
	kdtree_.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

	double corr_dst;
	// for 3d
	corr_dst = sqrt( pow( (source->points[i].x - target->points[ pointIdxNKNSearch[0] ].x), 2) +
					 pow( (source->points[i].y - target->points[ pointIdxNKNSearch[0] ].y), 2) +
					 pow( (source->points[i].z - target->points[ pointIdxNKNSearch[0] ].z), 2) );

	if( corr_dst < threshold_dst){
	  //add correspond(source, target)
	  std::pair<int, int> cor(i, pointIdxNKNSearch[0]);
	  new_correspond.push_back(cor);
	}
  }

  //duplicate check
  if(new_correspond.size() > 1){
  	std::cerr << "duplicate check ... size : " << new_correspond.size();
  	std::sort(new_correspond.begin(), new_correspond.end(), [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs){ return lhs.second < rhs.second; });

  	for(int i = 0; i < new_correspond.size() - 1; i++){
  	  if(new_correspond.at(i).second == new_correspond.at(i+1).second){
  		double score1  = sqrt( pow( (source->points[new_correspond.at(i).first].x - target->points[new_correspond.at(i).second].x), 2) +
  							   pow( (source->points[new_correspond.at(i).first].y - target->points[new_correspond.at(i).second].y), 2) +
  							   pow( (source->points[new_correspond.at(i).first].z - target->points[new_correspond.at(i).second].z), 2));

  		double score2 = sqrt( pow( (source->points[new_correspond.at(i+1).first].x - target->points[new_correspond.at(i+1).second].x), 2) +
  							  pow( (source->points[new_correspond.at(i+1).first].y - target->points[new_correspond.at(i+1).second].y), 2) +
  							  pow( (source->points[new_correspond.at(i+1).first].z - target->points[new_correspond.at(i+1).second].z), 2));
  		if(score1 >= score2){
  		  new_correspond.erase( new_correspond.begin() + i); //i番目さようなら
  		}
  		else{
  		  new_correspond.erase( new_correspond.begin() + i+1); //i+1番目さようなら
  		}
  	  }
  	}
  }
  std::cerr << " ---> " << new_correspond.size() << std::endl;

  //check  
  if(correspond_.size() == 0){
	num_of_new_point_ = new_correspond.size();
	stability_ = new_correspond.size();
	correspond_ = new_correspond;
	error_value_ = icp_4d::estimateEpsilon(target, source);
  }

  else{
	num_of_new_point_ = new_correspond.size() - correspond_.size(); 
	stability_ = estimateStability(new_correspond, correspond_);
	correspond_ = new_correspond;
	error_value_ = icp_4d::estimateEpsilon(target, source);
  }

  std::cerr << "estimate correspond done ... corr_size = " << correspond_.size() << std::endl;
  std::cerr << " epsilon = " << error_value_ << std::endl;
  std::cerr << " number of new associated points = " << num_of_new_point_ << std::endl;
  std::cerr << " stability = " << stability_ << std::endl;
}


void icp_4d::estimateCorrespond_4d(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
								   pcl::PointCloud<pcl::PointXYZI>::Ptr source,
								   double intensity_scale,
								   double threshold_dst){

  //create kdtree object
  //pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  if(kdtree_.getInputCloud () == 0){
	kdtree_.setInputCloud (target);
	std::cerr << "kdtree_stored(1)" << std::endl;
  }

  //new_correspond
  std::vector< std::pair<int, int> > new_correspond;

  int K = 1; //only nearest neighbor
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for(int i = 0; i < source->size(); i++){
	pcl::PointXYZI searchPoint(source->points[i]);
	kdtree_.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

	double corr_dst;
	// for 4d
	corr_dst = sqrt( pow( (source->points[i].x - target->points[ pointIdxNKNSearch[0] ].x), 2) +
					 pow( (source->points[i].y - target->points[ pointIdxNKNSearch[0] ].y), 2) +
					 pow( (source->points[i].z - target->points[ pointIdxNKNSearch[0] ].z), 2) +
					 pow( (source->points[i].intensity - target->points[ pointIdxNKNSearch[0] ].intensity) * intensity_scale, 2));

	if( corr_dst < threshold_dst){
	  //add correspond(source, target)
	  std::pair<int, int> cor(i, pointIdxNKNSearch[0]);
	  new_correspond.push_back(cor);
	}
  }

  //duplicate check
  if(new_correspond.size() > 1){
  	std::cerr << "duplicate check ... size : " << new_correspond.size();
  	std::sort(new_correspond.begin(), new_correspond.end(), [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs){ return lhs.second < rhs.second; });

  	for(int i = 0; i < new_correspond.size() - 1; i++){
  	  if(new_correspond.at(i).second == new_correspond.at(i+1).second){
  		double score1  = sqrt( pow( (source->points[new_correspond.at(i).first].x - target->points[new_correspond.at(i).second].x), 2) +
  							   pow( (source->points[new_correspond.at(i).first].y - target->points[new_correspond.at(i).second].y), 2) +
  							   pow( (source->points[new_correspond.at(i).first].z - target->points[new_correspond.at(i).second].z), 2));

  		double score2 = sqrt( pow( (source->points[new_correspond.at(i+1).first].x - target->points[new_correspond.at(i+1).second].x), 2) +
  							  pow( (source->points[new_correspond.at(i+1).first].y - target->points[new_correspond.at(i+1).second].y), 2) +
  							  pow( (source->points[new_correspond.at(i+1).first].z - target->points[new_correspond.at(i+1).second].z), 2));
  		if(score1 >= score2){
  		  new_correspond.erase( new_correspond.begin() + i); //i番目さようなら
  		}
  		else{
  		  new_correspond.erase( new_correspond.begin() + i+1); //i+1番目さようなら
  		}
  	  }
  	}
  }
  std::cerr << " ---> " << new_correspond.size() << std::endl;

  //check  
  if(correspond_.size() == 0){
	num_of_new_point_ = new_correspond.size();
	stability_ = new_correspond.size();
	correspond_ = new_correspond;
	error_value_ = icp_4d::estimateEpsilon(target, source);
  }

  else{
	num_of_new_point_ = new_correspond.size() - correspond_.size(); 
	stability_ = estimateStability(new_correspond, correspond_);
	correspond_ = new_correspond;
	error_value_ = icp_4d::estimateEpsilon(target, source);
  }

  std::cerr << "estimate correspond done ... corr_size = " << correspond_.size() << std::endl;
  std::cerr << " epsilon = " << error_value_ << std::endl;
  std::cerr << " number of new associated points = " << num_of_new_point_ << std::endl;
  std::cerr << " stability = " << stability_ << std::endl;

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
  //std::cerr << "estimate epsilon done ... epsilon=" << epsilon << std::endl;
  return epsilon;
}


int icp_4d::estimateStability(std::vector< std::pair<int, int> > new_correspond,
							  std::vector< std::pair<int, int> > old_correspond){
  int num_of_update = 0;
  if(new_correspond.size() > old_correspond.size()){
	int new_cor_id = 0;
	for(int old_cor_id = 0; old_cor_id < old_correspond.size(); old_cor_id++){
	  while(new_correspond.at(new_cor_id).first == old_correspond.at(old_cor_id).first){
		new_cor_id++;
	  }
	  if(new_correspond.at(new_cor_id).first != old_correspond.at(old_cor_id).first)
		num_of_update++;
	}
	num_of_update += (new_correspond.size() - old_correspond.size());
  }

  else if(new_correspond.size() == old_correspond.size()){
	int new_cor_id = 0;
	for(int old_cor_id = 0; old_cor_id < old_correspond.size(); old_cor_id++){
	  if(new_correspond.at(new_cor_id).first != old_correspond.at(old_cor_id).first)
		num_of_update++;
	}
  }

  else if(new_correspond.size() < old_correspond.size()){
	int old_cor_id = 0;
	for(int new_cor_id = 0; new_cor_id < new_correspond.size(); new_cor_id++){
	  while(new_correspond.at(new_cor_id).first == old_correspond.at(old_cor_id).first){
		old_cor_id++;
	  }
	  if(new_correspond.at(new_cor_id).first != old_correspond.at(old_cor_id).first)
		num_of_update++;
	}
	num_of_update += (old_correspond.size() - new_correspond.size());
  }

  std::cerr << "stability " << num_of_update << std::endl;
  return num_of_update;
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

  //apply SVD(4dicp paper)
  Eigen::Matrix3f mat_h = Eigen::Matrix3f::Identity();
  double Sxx = 0.0, Sxy = 0.0, Sxz = 0.0;
  double Syx = 0.0, Syy = 0.0, Syz = 0.0;
  double Szx = 0.0, Szy = 0.0, Szz = 0.0;
  for(int i = 0; i < correspond_.size(); i++){
  	//sampling
  	Eigen::Vector3f source_subtracted
  	  ( source->points[ correspond_.at(i).first ].x - source_mean(0),
  		source->points[ correspond_.at(i).first ].y - source_mean(1),
  		source->points[ correspond_.at(i).first ].z - source_mean(2) );

  	Eigen::Vector3f target_subtracted
  	  ( target->points[ correspond_.at(i).second ].x - target_mean(0),
  		target->points[ correspond_.at(i).second ].y - target_mean(1),
  		target->points[ correspond_.at(i).second ].z - target_mean(2) );

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

  mat_h << Sxx, Sxy, Sxz,
  	       Syx, Syy, Syz,
  	       Szx, Szy, Szz;

  Eigen::Matrix3f mat_h_inv = mat_h.inverse();

  // std::cerr << "mat H:" << std::endl
  // 			<< mat_h << std::endl;
  // std::cerr << "mat H(inv):" << std::endl
  // 			<< mat_h_inv << std::endl;

  Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> > svd(mat_h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::MatrixUType& u = svd.matrixU();
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::MatrixVType& v = svd.matrixV();
  const Eigen::JacobiSVD< Eigen::Matrix<float, 3, 3> >::SingularValuesType& sv = svd.singularValues();
  // std::cerr << "singular values" << std::endl
  // 			<< sv << std::endl;
  // std::cerr << "matrix V" << std::endl << v << std::endl;
  // std::cerr << "matrix U" << std::endl << u << std::endl;
  Eigen::Matrix3f ut = u.transpose();
  Eigen::Matrix3f vt = v.transpose();
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  double h22 = v.determinant() * ut.determinant();
  H << 1, 0, 0,
  	   0, 1, 0,
  	   0, 0, h22;
  //rotation_guess_ = v * ut; //<-間違い
  rotation_guess_ = u * H * vt; //<-鏡映防止
  //rotation_guess_ = u * vt; //<-standard
  translate_guess_ = target_mean - rotation_guess_ * source_mean;

  // std::cerr << "rotation:" << std::endl
  // 			<< rotation_guess_.matrix() << std::endl;
  // std::cerr << "translation: " << std::endl
  // 			<< translate_guess_ << std::endl;

  transform_guess_.block(0, 0, 3, 3) = rotation_guess_;
  transform_guess_.block(0, 3, 3, 1) = translate_guess_;
  transform_guess_(3, 0) = transform_guess_(3, 1) = transform_guess_(3, 2) = 0;
  transform_guess_(3, 3) = 1;
  std::cerr << "transform: " << std::endl
			<< transform_guess_ << std::endl;
}


void icp_4d::applyFastICP(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						  pcl::PointCloud<pcl::PointXYZI>::Ptr source){
  //error avoidance
  if(target->size() == 0 || source->size() == 0){
  	std::cerr << "[FastICP] error! no point cloud data!" << std::endl;
  	exit(-1);
  }

  //preparation
  clock_t startTime , endTime; double margin = 0.0; //時間計測用変数
  startTime = clock();
  const int target_size = target->points.size();
  const int source_size = source->points.size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr adjusted_target(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr adjusted_source(new pcl::PointCloud<pcl::PointXYZI>);

  //move source and target
  Eigen::AngleAxisf no_rotation (0.0, Eigen::Vector3f::UnitZ ());
  //targetの重心を原点に合わせる
  double mean_x_t = 0.0, mean_y_t = 0.0, mean_z_t = 0.0;
  for(int i = 0; i < target_size; i++){
  	double x = target->points.at(i).x;
  	double y = target->points.at(i).y;
  	double z = target->points.at(i).z;
  	mean_x_t += x;
  	mean_y_t += y;
  	mean_z_t += z;
  }
  mean_x_t /= target_size;
  mean_y_t /= target_size; 
  mean_z_t /= target_size; 
  Eigen::Translation3f center_translation_t (-mean_x_t, -mean_y_t, -mean_z_t);
  Eigen::Matrix4f to_center_guess_t = (center_translation_t * no_rotation).matrix ();
  pcl::transformPointCloud(*target, *adjusted_target, to_center_guess_t);

  //sourceの重心を原点に合わせる
  double mean_x_s = 0.0, mean_y_s = 0.0, mean_z_s = 0.0;
  for(int i = 0; i < source_size; i++){
  	double x = source->points.at(i).x;
  	double y = source->points.at(i).y;
  	double z = source->points.at(i).z;
  	mean_x_s += x;
  	mean_y_s += y;
  	mean_z_s += z;
  }
  mean_x_s /= source_size;
  mean_y_s /= source_size; 
  mean_z_s /= source_size; 
  Eigen::Translation3f center_translation_s (-mean_x_s, -mean_y_s, -mean_z_s);
  Eigen::Matrix4f to_center_guess_s = (center_translation_s * no_rotation).matrix ();
  pcl::transformPointCloud(*source, *adjusted_source, to_center_guess_s);

  //estimate translate
  translate_guess_(0) = mean_x_t - mean_x_s;
  translate_guess_(1) = mean_y_t - mean_y_s;
  translate_guess_(2) = mean_z_t - mean_z_s;
  std::cerr << "[FastICP] calculating translate ... OK!" << std::endl;  

  //pointcloud ---> Eigen::Matrix
  Eigen::MatrixXd target_mat(3, target_size);
  Eigen::MatrixXd source_mat(3, source_size);
  // store
  for(int i = 0; i < target_size; i++){
	target_mat(0,i) = adjusted_target->points.at(i).x;
	target_mat(1,i) = adjusted_target->points.at(i).y;
	target_mat(2,i) = adjusted_target->points.at(i).z;
  }
  for(int i = 0; i < source_size; i++){
	source_mat(0,i) = adjusted_source->points.at(i).x;
	source_mat(1,i) = adjusted_source->points.at(i).y;
	source_mat(2,i) = adjusted_source->points.at(i).z;
  }
  std::cerr << "[FastICP] storing to Eigen ... OK!" << std::endl;

  //svd target
  Eigen::JacobiSVD< Eigen::MatrixXd > svd_t(target_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  std::cerr << "[FastICP] Applying SVD about target data ... OK!" << std::endl;

  //const Eigen::JacobiSVD< Eigen::MatrixXd >::MatrixUType& u_t = svd_t.matrixU();
  Eigen::MatrixXd U_t = svd_t.matrixU();
  Eigen::Matrix3f U_t3;
  U_t3(0,0) = U_t(0,0); U_t3(0,1) = U_t(0,1); U_t3(0,2) = U_t(0,2);  
  U_t3(1,0) = U_t(1,0); U_t3(1,1) = U_t(1,1); U_t3(1,2) = U_t(1,2);  
  U_t3(2,0) = U_t(2,0); U_t3(2,1) = U_t(2,1); U_t3(2,2) = U_t(2,2);  

  std::cerr << "[FastICP] convert MatrixXd to Matrix3f ... OK!" << std::endl;  

  //svd source
  Eigen::JacobiSVD< Eigen::MatrixXd > svd_s(source_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  std::cerr << "[FastICP] Applying SVD about source data ... OK!" << std::endl;
  Eigen::MatrixXd U_s = svd_s.matrixU();
  Eigen::Matrix3f U_s3;
  U_s3(0,0) = U_s(0,0); U_s3(0,1) = U_s(0,1); U_s3(0,2) = U_s(0,2);  
  U_s3(1,0) = U_s(1,0); U_s3(1,1) = U_s(1,1); U_s3(1,2) = U_s(1,2);  
  U_s3(2,0) = U_s(2,0); U_s3(2,1) = U_s(2,1); U_s3(2,2) = U_s(2,2);

  std::cerr << "[FastICP] convert MatrixXd to Matrix3f ... OK!" << std::endl;  

  //estimate rotate
  Eigen::Matrix3f tU_t3 = U_t3.transpose();
  Eigen::Matrix3f tU_s3 = U_s3.transpose();
  rotation_guess_ = U_t3 * tU_s3;
  std::cerr << "[FastICP] calculating rotation ... OK!" << std::endl;  

  endTime = clock();
  //show result & save
  transform_guess_.block(0, 0, 3, 3) = rotation_guess_;
  transform_guess_.block(0, 3, 3, 1) = translate_guess_;
  transform_guess_(3, 0) = transform_guess_(3, 1) = transform_guess_(3, 2) = 0;
  transform_guess_(3, 3) = 1;
  std::cerr << "transform: " << std::endl
  			<< transform_guess_ << std::endl;
  std::cerr << "[FastICP] processing time:" << (double)(endTime - startTime)/CLOCKS_PER_SEC << "秒" << endl;
}


void icp_4d::applyICP(double max_range,
					  double max_intensity,
					  double threshold_radius,
					  Eigen::Matrix4f init_guess){

  if(target_cloud_->size() == 0 || source_cloud_->size() == 0){
	std::cerr << "[apply ICP] no data!" << std::endl;
	exit(-1);
  }

  std::cerr << "target size: " << target_cloud_->size() << std::endl;  
  std::cerr << "source size: " << source_cloud_->size() << std::endl;

  //initialize 
  error_value_ = FLT_MAX;
  num_of_new_point_ = INT_MAX;
  stability_ = INT_MAX;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_transform (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud (*source_cloud_, *source_transform, init_guess);

  //pretreatment
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_target (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_source (new pcl::PointCloud<pcl::PointXYZI>);
  icp_4d::setMaxRange(circle_target, target_cloud_, max_range);
  icp_4d::setMaxRange(circle_source, source_transform, max_range);

  pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_target (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr normalized_source (new pcl::PointCloud<pcl::PointXYZI>);
  icp_4d::normalizePointCloud(normalized_target, circle_target, max_range, max_intensity);
  icp_4d::normalizePointCloud(normalized_source, circle_source, max_range, max_intensity);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  int i = 0;
  while(i != 100){
	//
	icp_4d::estimateCorrespond(normalized_target, normalized_source, threshold_radius);
	icp_4d::estimateTransform(normalized_target, normalized_source);
	//normalized_source update
	pcl::transformPointCloud (*normalized_source, *normalized_source, transform_guess_);
	i++;

	if(i == 99){
	  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity1(normalized_target, "intensity");
	  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity2(normalized_source, "intensity");
	  
	  if( !viewer->updatePointCloud (normalized_target, intensity1, "target cloud") ){
		viewer->addPointCloud<pcl::PointXYZI> (normalized_target, intensity1, "target cloud");
	  }
	  
	  Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
	  Eigen::Translation3f init_translation (2.5, 0.0, 0.0);
	  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
	  pcl::PointCloud<pcl::PointXYZI>::Ptr source_for_viz(new pcl::PointCloud<pcl::PointXYZI>);
	  pcl::transformPointCloud (*normalized_source, *source_for_viz, init_guess);
	  
	  if( !viewer->updatePointCloud (source_for_viz, intensity2, "input cloud") ){
		viewer->addPointCloud<pcl::PointXYZI> (source_for_viz, intensity2, "input cloud");
	  }
	  
	  for(int n = 0; n < correspond_.size(); n++){
		std::stringstream line_id;
		line_id << "line_" << i << "_" << n;
		viewer->addLine<pcl::PointXYZI> (source_for_viz->points[ correspond_.at(n).first ],
										 normalized_target->points[ correspond_.at(n).second ],
										 1.0,
										 0.0,
										 0.0,
										 line_id.str());
		if(n > 500){
		  break;
		}
	  }
	  while(!viewer->wasStopped ()){	  
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
	}
  }
}
