//ROS Node that provides an estimate of each objects initial pose.
//If colourTracker is running, then the positions can be estimated
//from this. Otherwise, the initial pose is estimated after HMP 
//detection.

//TODO: Error handling

#include "poseInitializer.hpp"

PoseInitializerNode::PoseInitializerNode(std::vector<std::string> file_paths, int num_objects, std::string background_removal_service, std::string frame_id) : node_handle_("~"), frame_id_(frame_id) {
    num_objects_ = num_objects;
    background_removal_service_ = background_removal_service;
    for (int i=0; i < num_objects_; i++) {
        pcl::PointCloud<pcl::PointXYZ> file_mesh;
        int ply_version;
        pcl::PLYReader reader;
        std::cout << "Loading file " << file_paths[i] << "\n";
        std::cout << "Code: " << reader.read(file_paths[i],
		    file_mesh,
		    //Eigen::Vector4f(0,0,0,0),
		    //Eigen::Quaternionf(0,0,0,1),
		    //ply_version,
            0) << "\n";

        //pcl::io::loadPLYFile(file_paths[i], file_mesh);
        file_meshes_.push_back(file_mesh);
    }
    initial_poses_publisher_ =
        node_handle_.advertise<geometry_msgs::PoseArray>
            ("initial_poses", 0);
}


void PoseInitializerNode::hmp_callback(const object_assembly_msgs::Points2D& points) {
    //std::cout << "Received message from hmp\n";
    ros::ServiceClient client = node_handle_.serviceClient<object_assembly_msgs::FetchPCLsGivenLocations>(background_removal_service_);
    object_assembly_msgs::FetchPCLsGivenLocations srv;
    srv.request.num_objects = 4;
    srv.request.points = points;
    object_assembly_msgs::PointCloudArray pcl_array;
    if (client.call(srv))
    {
        pcl_array = srv.response.point_cloud_array;
    }
    else
    {
        ROS_ERROR("Failed to call background removal service");
        return;
    }
    for (int i = 0; i < pcl_array.pointclouds.size(); i++)
        if (pcl_array.pointclouds[i].height*pcl_array.pointclouds[i].width == 0) return;

    //try icp from different orientations and choose best
    calculate_icp(pcl_array);
    publish();
    initial_poses_.clear();
}


void PoseInitializerNode::colour_tracker_callback(const object_assembly_msgs::Points2D& points) {
    //std::cout << "Received message from colour tracker\n";
    ros::ServiceClient client = node_handle_.serviceClient<object_assembly_msgs::FetchPCLsGivenLocations>(background_removal_service_);
    object_assembly_msgs::FetchPCLsGivenLocations srv;
    srv.request.num_objects = 4;
    srv.request.points = points;
    object_assembly_msgs::PointCloudArray pcl_array;
    if (client.call(srv))
    {
        pcl_array = srv.response.point_cloud_array;
    }
    else
    {
        ROS_ERROR("Failed to call background removal service");
        return;
    }
    for (int i = 0; i < pcl_array.pointclouds.size(); i++)
        if (pcl_array.pointclouds[i].height*pcl_array.pointclouds[i].width == 0) return;

    //try icp from different orientations and choose best
    calculate_icp(pcl_array);
    publish();
    initial_poses_.clear();
}


void PoseInitializerNode::calculate_icp(object_assembly_msgs::PointCloudArray &pcl_array) {
    //std::cout << "Calculating ICP\n";
    for (int i = 0; i < pcl_array.pointclouds.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::fromROSMsg(pcl_array.pointclouds[i], cloud_in);
        double best_score = INFINITY;
        tf::Transform detected_pose;
        for (int j = 0; j < num_random_rotations_; j++) {
            Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

            //Random rotation
            Eigen::Quaternionf rand_rot(std::rand(),
                                       std::rand(),
                                       std::rand(),
                                       std::rand());
            rand_rot.normalize();
            transform_2.rotate(rand_rot);

            //Move mesh to mean of target point cloud
            Eigen::Matrix<float, 3, 3> covariance_matrix;
		    Eigen::Matrix<float, 4, 1> centroid;
            pcl::computeMeanAndCovarianceMatrix(cloud_in, covariance_matrix, centroid); 	
            Eigen::Affine3f transform_3;
            transform_3 = Eigen::Translation3f(centroid.head(3));
            transform_2 = transform_3 * transform_2;
            
            pcl::PointCloud<pcl::PointXYZ> mesh_rand_rot;
            pcl::transformPointCloud(file_meshes_[i], mesh_rand_rot,
                                      transform_2);
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);
            *pcl_in = mesh_rand_rot;
            icp.setInputSource(pcl_in);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target(new pcl::PointCloud<pcl::PointXYZ>);
            *pcl_target = cloud_in;
            icp.setInputTarget(pcl_target);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
            //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            //std::cout << icp.getFinalTransformation() << std::endl;
            if (icp.getFitnessScore() < best_score) {
                best_score = icp.getFitnessScore();
                //std::cout << "Getting transform matrix\n";
                Eigen::Matrix<float, 4, 4> T =
                        icp.getFinalTransformation();
                T = T * transform_2.matrix();
                //std::cout << "Filling rotation matrix\n";
                tf::Matrix3x3 rotation(T(0,0), T(0,1), T(0,2),
                                       T(1,0), T(1,1), T(1,2),
                                       T(2,0), T(2,1), T(2,2));
                //std::cout << "Updating detected pose\n";
                detected_pose = tf::Transform(rotation,
                                              tf::Vector3(T(0,3),
                                                          T(1,3),
                                                          T(2,3)));
            }
        }
        initial_poses_.push_back(detected_pose);
    }
}

void PoseInitializerNode::publish() {
    geometry_msgs::PoseArray msg;
    for (int i = 0; i < num_objects_; i++) {
        geometry_msgs::Pose single_pose;
        tf::poseTFToMsg (initial_poses_[i], single_pose);
        msg.poses.push_back(single_pose);
    }
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    initial_poses_publisher_.publish(msg);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pose_initializer");
    ros::NodeHandle nh("~");

    std::string camera_topic;
    nh.getParam("camera_topic", camera_topic);
    std::string colour_tracker_topic;
    nh.getParam("colour_tracker_topic", colour_tracker_topic);
    std::string background_removal_service;
    nh.getParam("background_removal_service",
                background_removal_service);
    std::string frame_id;
    nh.getParam("frame_id", frame_id);

    int num_objects;
    std::vector<std::string> file_names;
    std::string directory;
    nh.getParam("filenames", file_names);
    nh.getParam("directory", directory);
    std::vector<std::string> file_paths;
    num_objects = file_names.size();
    for (int i = 0; i < num_objects; i++) {
        file_paths.push_back(directory + "/" + file_names[i]); 
    }

    std::cout << "Creating Pose Initialize Node...\n";
    PoseInitializerNode pose_initializer_node(file_paths, num_objects, background_removal_service, frame_id);
    std::cout << "Created Pose Initialize Node\n";
    ros::Subscriber subscriber_hmp = 
        nh.subscribe(camera_topic,
                     1,
                     &PoseInitializerNode::hmp_callback,
                     &pose_initializer_node);

    ros::Subscriber subscriber_colour_tracker = 
        nh.subscribe(colour_tracker_topic,
                     1,
                     &PoseInitializerNode::colour_tracker_callback,
                     &pose_initializer_node);
//std::cout << "Waiting for message...\n";
    ros::spin();
}
