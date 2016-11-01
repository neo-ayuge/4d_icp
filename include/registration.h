#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

class icp_4d{
  //member-----------------------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_;
  std::vector< std::pair<int, int> > correspond_;
  Eigen::Matrix4f transform_guess_;
  Eigen::Vector3f translate_guess_;
  Eigen::Matrix3f rotation_guess_;
  //---------------------------------//member

 public:
  //setter
  void setTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud);
  void setSource(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud);

  //getter
  pcl::PointCloud<pcl::PointXYZI>::Ptr getTarget();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getSource();

  void setMaxRange(pcl::PointCloud<pcl::PointXYZI>::Ptr output,
				   pcl::PointCloud<pcl::PointXYZI>::ConstPtr source, 
				   double max_range);
  
  void normalizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output,
						   pcl::PointCloud<pcl::PointXYZI>::ConstPtr source,
						   double max_range,
						   double max_intensity);

  void estimateCorrespond(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						  pcl::PointCloud<pcl::PointXYZI>::Ptr source,
						  double threshold_dst,
						  double max_range);

  double estimateEpsilon(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						 pcl::PointCloud<pcl::PointXYZI>::Ptr source);

  void estimateTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						 pcl::PointCloud<pcl::PointXYZI>::Ptr source);

  void applyICP(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
				pcl::PointCloud<pcl::PointXYZI>::Ptr source,
				double max_range,
				double max_intensity,
				double threshold_dst);
};






