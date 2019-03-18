#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <vector> 
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <object_assembly_msgs/Point2D.h>
#include <object_assembly_msgs/Points2D.h>
#include <object_assembly_msgs/PointCloudArray.h>
#include <object_assembly_msgs/FetchPCLsGivenLocations.h>
#include <object_assembly_msgs/FetchPCLs.h>
#include <object_assembly_msgs/FetchSurfaceParams.h>
#include <object_assembly_msgs/FetchForegroundMask.h>
//#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "dbscan.h"

using namespace cv;

struct CropInfo {
    double top_margin;
    double left_margin;
    double width;
    double height;
};

class BackgroundRemovalNode {

public:

    BackgroundRemovalNode(float max_depth, float surf_thresh, bool recalculate_surface_params, std::string clustering_type, CropInfo crop_info, bool crop_for_table_detection, bool show_table_mask);

    void CameraInfoCallback(const sensor_msgs::CameraInfo &cinfo);

    void callback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::PointCloud2ConstPtr& pointcloud);

    bool fetch_pcls_given_locations_service(
            object_assembly_msgs::FetchPCLsGivenLocations::Request &req,
            object_assembly_msgs::FetchPCLsGivenLocations::Response &res);

    bool fetch_pcls_service(
            object_assembly_msgs::FetchPCLs::Request &req,
            object_assembly_msgs::FetchPCLs::Response &res);

    bool fetch_surface_params_service(
            object_assembly_msgs::FetchSurfaceParams::Request &req,
            object_assembly_msgs::FetchSurfaceParams::Response &res);

    bool fetch_foreground_mask(
            object_assembly_msgs::FetchForegroundMask::Request &req,
            object_assembly_msgs::FetchForegroundMask::Response &res);

    void process_inputs();

    void response();

    void GetTableImage();

    int GetFlatSurfaceParams();

    int ImageSegmentation();

    int ImageSegmentation(object_assembly_msgs::Points2D points);

    void createOutputPCLs();

    void calculate_cropped_range();

    bool in_cropped_frame(int index);

private:
    float SURF_THRESH_;// = 0.97;
    float MAX_DEPTH_;// = 2000;
    int NUM_OBJECTS_ = 4;
    int KMEANS_NUM_ATTEMPTS_ = 3;
    std::string clustering_type_;
    //Pointers to the topic contents
    sensor_msgs::ImageConstPtr depth_image_ptr_;
    sensor_msgs::PointCloud2ConstPtr pointcloud_ptr_;
    //Depth image and PCL stored when needed
    Mat depth_image_, table_mask_;
    pcl::PointCloud<pcl::PointXYZ> input_pointcloud_;
    //Output PCLs (cropped around objects)
    //object_assembly_msgs::PointCloudArray output_pointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> output_pointclouds_;
    int height_;
    int width_;
    Mat table_image_;
    Mat labels_;
    std::vector<int> foreground_ind_;
    float surface_params_[3];
    bool topics_received_ = false;
    bool surface_params_found_ = true;
    CropInfo crop_info_;
    bool crop_image_;
    int min_cropped_x_;
    int min_cropped_y_;
    int max_cropped_x_;
    int max_cropped_y_;
    bool recalculate_surface_params_;
    bool crop_for_table_detection_;
    bool show_table_mask_;

    ros::NodeHandle node_handle_;
    ros::Publisher pointclouds_publisher_;
    ros::Publisher resized_depth_publisher_;
    ros::Publisher newcamerainfo_publisher_;

    sensor_msgs::CameraInfo cinfo_;
};
