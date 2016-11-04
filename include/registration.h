#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <cmath>
#include <float.h>
#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

class icp_4d{
  //member-----------------------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
  std::vector< std::pair<int, int> > correspond_;
  Eigen::Matrix4f transform_guess_;
  Eigen::Vector3f translate_guess_;
  Eigen::Matrix3f rotation_guess_;

  //convergence criteria
  //1) error value -> 0
  //2) number of associated new points -> 0
  //3) association stability measure -> 0
  double error_value_;
  int num_of_new_point_;
  int stability_;
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

  int estimateStability(std::vector< std::pair<int, int> > new_correspond,
						std::vector< std::pair<int, int> > old_correspond);

  void estimateTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr target, 
						 pcl::PointCloud<pcl::PointXYZI>::Ptr source);

  void applyICP(double max_range,
  				double max_intensity,
  				double threshold_radius,
				Eigen::Matrix4f init_guess);

};






