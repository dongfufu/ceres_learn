
#include  <pcl-1.8/pcl/visualization/pcl_plotter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include <vector>
#include<utility>
#include <iostream>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include<chrono>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using namespace std;
typedef pcl::PointXYZ PointT;

struct CostFun{
    
    CostFun(double x,double y,double z):x_(x),y_(y),z_(z){}
    
    template<typename T>
    bool operator()(const T* const parm,T* residual) const{
        residual[0]=pow((x_-parm[0]),2)+pow((y_-parm[1]),2)+pow((z_-parm[2]),2)-pow(parm[3]*(x_-parm[0])+parm[4]*(y_-parm[1])+parm[5]*(z_-parm[2]),2)-pow(parm[6],2);
        return true;
    }
    const double x_,y_,z_;
};




int main (int argc, char** argv)
{
   // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  reader.read ("/home/dongfu/projects/Cylinder_extract/210127_101328_height.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1000,1000);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  std::cout<<"normals size="<<cloud_normals->size()<<endl;
  // Create the segmentation object for the planar model and set all the parameters
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//   seg.setNormalDistanceWeight (0.1);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   seg.setDistanceThreshold (0.5);
//   seg.setInputCloud (cloud_filtered);
//   seg.setInputNormals (cloud_normals);
//   // Obtain the plane inliers and coefficients
//   seg.segment (*inliers_plane, *coefficients_plane);
//   std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
// 
//   // Extract the planar inliers from the input cloud
//   extract.setInputCloud (cloud_filtered);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (false);
// 
//   // Write the planar inliers to disk
//   pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//   extract.filter (*cloud_plane);
//   std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//   writer.write ("/home/dongfu/projects/Cylinder_extract/table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
// 
//   // Remove the planar inliers, extract the rest
//   extract.setNegative (true);
//   extract.filter (*cloud_filtered2);
//   extract_normals.setNegative (true);//是否设置负
//   extract_normals.setInputCloud (cloud_normals);
//   extract_normals.setIndices (inliers_plane);
//   extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (5);
  seg.setRadiusLimits (0, 600);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
 
  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
	  writer.write ("/home/dongfu/projects/Cylinder_extract/cylinder.pcd", *cloud_cylinder, false);
  }
  
  //剩余的点
   pcl::PointCloud<PointT>::Ptr cloud_filter3 (new pcl::PointCloud<PointT> ());
   extract.setNegative(true);
   extract.setInputCloud (cloud_filtered);
   extract.setIndices (inliers_cylinder);
   extract.filter (*cloud_filter3);
   cout<<"剩余点的个数为"<<cloud_filter3->size()<<endl;
  
  //设置优化初值
  double x0=coefficients_cylinder->values[0];
  double y0=coefficients_cylinder->values[1];
  double z0=coefficients_cylinder->values[2];
  double a=coefficients_cylinder->values[3];
  double b=coefficients_cylinder->values[4];
  double c=coefficients_cylinder->values[5];
  double r0=coefficients_cylinder->values[6];
  
     //计算优化后的参数误差
//    double sum_error=0;
//    vector<double> error;
//    double ave_error=0;
//   for(int i=0;i<cloud_cylinder->size();i++)
//   {
//      double temp1=pow(cloud_cylinder->points[i].x-x0,2)+pow(cloud_cylinder->points[i].y-y0,2)+pow(cloud_cylinder->points[i].z-z0,2);
//      double temp2=pow(a*(cloud_cylinder->points[i].x-x0)+b*(cloud_cylinder->points[i].y-y0)+c*(cloud_cylinder->points[i].z-z0),2);
//      double r_predict_2=temp1-temp2;
//      double r_predict=sqrt(r_predict_2);
//      double temp_error=r0-r_predict;
//      double temp_error_2=pow(temp_error,2);
//      sum_error+=temp_error_2;
//      error.push_back(temp_error);
//  }
//  ave_error=sqrt(sum_error/cloud_cylinder->size());
//  cout<<"平均误差为"<<ave_error<<endl;
//  cout<<"总误差"<<sum_error<<endl;
 
 //开始优化
  //设置参数块
  double param[7]={x0,y0,z0,a,b,c,r0};
  //定义优化问题
  ceres::Problem problem;
  for(int i=0;i<cloud_cylinder->size();i++)
  {
     problem.AddResidualBlock(
         new ceres::AutoDiffCostFunction<CostFun,1,7>(new CostFun(cloud_cylinder->points[i].x,cloud_cylinder->points[i].y,cloud_cylinder->points[i].z)),
        nullptr,
        param
     );
  }
  //配置求解器
  ceres::Solver::Options options;
  options.linear_solver_type=ceres::DENSE_NORMAL_CHOLESKY;//增量方程求解
  options.minimizer_progress_to_stdout=true;//输出到cout
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  cout<<summary.BriefReport()<<endl;
  cout<<"优化之后的结果为:"<<endl;
  for(auto x:param)
  {
      cout<<x<<" ";
  }
  cout<<std::endl;
  cout<<"优化后半径为"<<param[6]<<endl;
  
  
 
   
   //计算优化后的参数误差
   double r_=param[6];
   double sum_error=0;
   vector<double> error;
   double ave_error=0;
  for(int i=0;i<cloud_cylinder->size();i++)
  {
     double temp1=pow(cloud_cylinder->points[i].x-param[0],2)+pow(cloud_cylinder->points[i].y-param[1],2)+pow(cloud_cylinder->points[i].z-param[2],2);
     double temp2=pow(param[3]*(cloud_cylinder->points[i].x-param[0])+param[4]*(cloud_cylinder->points[i].y-param[1])+param[5]*(cloud_cylinder->points[i].z-param[2]),2);
     double r_predict_2=temp1-temp2;
     double r_predict=sqrt(r_predict_2);
     double temp_error=r_-r_predict;
     double temp_error_2=pow(temp_error,2);
     sum_error+=temp_error_2;
     error.push_back(temp_error);
 }
 ave_error=sqrt(sum_error/cloud_cylinder->size());
 cout<<"平均误差为"<<ave_error<<endl;
 //cout<<"总误差"<<sum_error<<endl;
   
   //绘制图形
 vector<double> index;
 for(int i=0;i<cloud_cylinder->size();i++)
 {
     index.push_back(i);
}
 
 //pcl_plotter
  pcl::visualization::PCLPlotter *plotter = new  pcl::visualization::PCLPlotter ("my plot");
  plotter->addPlotData(index,error,"error",vtkChart::POINTS);
  plotter->spinOnce(2000);
  //plotter->clearPlots();
  
  //show cloud 
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 25);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_cylinder, "cylinder cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cylinder cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud,cloud_normals, 15, 0.7, "normals");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }
  return (0);
}
