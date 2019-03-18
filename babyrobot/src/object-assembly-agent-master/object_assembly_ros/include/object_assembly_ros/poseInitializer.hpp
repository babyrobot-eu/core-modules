#include <stdlib.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector> 
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include "pcl_ros/point_cloud.h"
#include <object_assembly_msgs/Point2D.h>
#include <object_assembly_msgs/Points2D.h>
#include <object_assembly_msgs/PointCloudArray.h>
#include <object_assembly_msgs/FetchPCLsGivenLocations.h>
#include <object_assembly_msgs/FetchPCLs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry> 
#include <geometry_msgs/PoseArray.h>

class PoseInitializerNode {

public:
    PoseInitializerNode(std::vector<std::string> file_paths, int num_objects, std::string background_removal_service, std::string frame_id);

    void hmp_callback(const object_assembly_msgs::Points2D& points);

    void colour_tracker_callback(const object_assembly_msgs::Points2D& points);

    void calculate_icp(object_assembly_msgs::PointCloudArray &pcl_array);

    void publish();

private:
    int num_objects_;
    int num_random_rotations_ = 4;
    std::string frame_id_;
    std::string background_removal_service_;
    ros::NodeHandle node_handle_;
    ros::Publisher initial_poses_publisher_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> file_meshes_;
    std::vector<tf::Transform> initial_poses_;

};
