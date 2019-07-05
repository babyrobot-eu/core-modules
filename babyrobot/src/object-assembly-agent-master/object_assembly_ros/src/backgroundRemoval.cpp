#include "backgroundRemoval.hpp"

//TODO: remove initial values of surface_params_ and set surface_params_found_ to false when ready
BackgroundRemovalNode::BackgroundRemovalNode(float max_depth, float surf_thresh, bool recalculate_surface_params, std::string clustering_type, CropInfo crop_info, bool crop_for_table_detection, bool show_table_mask) : node_handle_("~"), clustering_type_(clustering_type) {
    pointclouds_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("TempPCL", 0);
    MAX_DEPTH_ = max_depth;
    SURF_THRESH_ = surf_thresh;
    recalculate_surface_params_ = recalculate_surface_params;
    crop_info_ = crop_info;
    crop_for_table_detection_ = crop_for_table_detection;
    show_table_mask_ = show_table_mask;
    surface_params_[0] = -0.021108340472;
    surface_params_[1] = 0.308822542429;
    surface_params_[2] = -0.585311472416;
    surface_params_found_ = false;
}

void BackgroundRemovalNode::callback(
        const sensor_msgs::ImageConstPtr& depth_image,
        const sensor_msgs::PointCloud2ConstPtr &pointcloud) {
    depth_image_ptr_ = depth_image;
    pointcloud_ptr_ = pointcloud;
    topics_received_ = true;
}


bool BackgroundRemovalNode::fetch_pcls_given_locations_service(
        object_assembly_msgs::FetchPCLsGivenLocations::Request &req,
        object_assembly_msgs::FetchPCLsGivenLocations::Response &res) {
    //std::cout << "Fetch PCLs service...\n";
    process_inputs();
    NUM_OBJECTS_ = req.num_objects;
    //std::cout << "Segmenting Image...\n";
    if (ImageSegmentation(req.points) == -1) return false;
    //std::cout << "Creating outputs...\n";
    createOutputPCLs();

    object_assembly_msgs::PointCloudArray pcl_array;
    //std::cout << "Converting outputs...\n";
    for (int i = 0; i < output_pointclouds_.size(); i++) {
        sensor_msgs::PointCloud2 temp_pcl;
        pcl::toROSMsg(output_pointclouds_[i], temp_pcl);
        temp_pcl.header.frame_id = "camera_depth_optical_frame";
        //if (i==0) pointclouds_publisher_.publish(temp_pcl);
        pcl_array.pointclouds.push_back(temp_pcl);
    }
    res.point_cloud_array = pcl_array;
    output_pointclouds_.clear();
    foreground_ind_.clear();
    depth_image_.release();
    table_image_.release();
    table_mask_.release();
    labels_.release();
    return true;
}


bool BackgroundRemovalNode::fetch_pcls_service(
        object_assembly_msgs::FetchPCLs::Request &req,
        object_assembly_msgs::FetchPCLs::Response &res) {
    //std::cout << "Fetch PCLs service...\n";
    process_inputs();
    NUM_OBJECTS_ = req.num_objects;
    //std::cout << "Segmenting Image...\n";
    if (ImageSegmentation() == -1) return false;
    //std::cout << "Creating outputs...\n";
    createOutputPCLs();

    object_assembly_msgs::PointCloudArray pcl_array;
    //std::cout << "Converting outputs...\n";
    for (int i = 0; i < output_pointclouds_.size(); i++) {
        sensor_msgs::PointCloud2 temp_pcl;
        pcl::toROSMsg(output_pointclouds_[i], temp_pcl);
        temp_pcl.header.frame_id = "camera_depth_optical_frame";
        //if (i==0) pointclouds_publisher_.publish(temp_pcl);
        pcl_array.pointclouds.push_back(temp_pcl);
    }
    res.point_cloud_array = pcl_array;
    output_pointclouds_.clear();
    foreground_ind_.clear();
    depth_image_.release();
    table_image_.release();
    table_mask_.release();
    labels_.release();
    return true;
}

bool BackgroundRemovalNode::fetch_surface_params_service(
        object_assembly_msgs::FetchSurfaceParams::Request &req,
        object_assembly_msgs::FetchSurfaceParams::Response &res) {
    if (!topics_received_) {
        ROS_INFO("NO CAMERA DATA RECEIVED");
        return false;
    }
    if (req.recalculate) {
        process_inputs();
        if (GetFlatSurfaceParams() == -1) {
            ROS_INFO("Error calculating surface parameters");
            return false;
        }
        else
            surface_params_found_ = true;
    }
    else {
        if (!surface_params_found_) {
            ROS_INFO("Surface parameters have not been calculated");
            return false;
        }
    }
    res.surface_parameters.a1 = surface_params_[0];
    res.surface_parameters.a2 = surface_params_[1];
    res.surface_parameters.a3 = surface_params_[2];
    return true;
}

bool BackgroundRemovalNode::fetch_foreground_mask(
        object_assembly_msgs::FetchForegroundMask::Request &req,
        object_assembly_msgs::FetchForegroundMask::Response &res) {
    std::vector<Point3f> foregroundCoords;
    std::vector<int> foreground_ind;
    int foreground_pixels_num = 0;
    //std::cout << "Separating foreground...\n";
    for (int i = 0; i < height_*width_; i++) {
        if (input_pointcloud_.points[i].y < (surface_params_[0]*input_pointcloud_.points[i].x + surface_params_[2]*input_pointcloud_.points[i].z + surface_params_[1]) - SURF_THRESH_ && input_pointcloud_.points[i].z < MAX_DEPTH_ && in_cropped_frame(i)) {
            
        }
    }
    return true;
}

void BackgroundRemovalNode::process_inputs() {
    try {
        cv_bridge::toCvShare(depth_image_ptr_)->image.copyTo(depth_image_);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.",
                  depth_image_ptr_->encoding.c_str());
    }
    height_ = depth_image_.rows;
    width_ = depth_image_.cols;
    pcl::fromROSMsg(*pointcloud_ptr_, input_pointcloud_);
    if (input_pointcloud_.height != height_ ||
        input_pointcloud_.width != width_) {
        resize(depth_image_, depth_image_,
            Size(input_pointcloud_.width, input_pointcloud_.height));
        height_ = depth_image_.rows;
        width_ = depth_image_.cols;
    }
    calculate_cropped_range();
}

void BackgroundRemovalNode::response() {
}


void BackgroundRemovalNode::GetTableImage() {
	//Perform distance transform on image gradient and threshold the result
    Rect r1(0, 0, width_, height_ - 1);
    Rect r2(0, 1, width_, height_ - 1);
    Mat img_diff = Mat(height_ - 1, width_, CV_16UC1);
    absdiff(depth_image_(r1), depth_image_(r2), img_diff);
    Mat notzero = img_diff!=0;
    dilate(notzero, notzero, getStructuringElement(MORPH_RECT, Size(4, 4)));
    Mat some_ones_and_zeros;
    if (crop_for_table_detection_ && crop_image_) {
        some_ones_and_zeros = Mat::zeros(height_ - 1, width_, CV_16UC1);
        Rect r3(min_cropped_x_, min_cropped_y_, max_cropped_x_ - min_cropped_x_, max_cropped_y_ - min_cropped_y_);
        Mat temp = some_ones_and_zeros(r3);
        Mat temp2 = Mat::ones(max_cropped_y_ - min_cropped_y_, max_cropped_x_ - min_cropped_x_, CV_16UC1);
        temp2.copyTo(temp);
    }
    else {
        some_ones_and_zeros = Mat::ones(height_ - 1, width_, CV_16UC1);
    }
    Mat cropped_range = some_ones_and_zeros > 0.5;
    Mat mask1 = img_diff < 500 & notzero & cropped_range;
    Mat dist;
    distanceTransform(mask1, dist, CV_DIST_L2, 3);
    normalize(dist, dist, 0, 1., NORM_MINMAX);
    Mat mask2 = dist > 0.5;
    
    //Find contour with largest area
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(mask2, contours, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    int maxAreaIndex = 0;
    double maxArea = -1;
    for (int i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) > maxArea) {
            maxArea = contourArea(contours[i]);
            maxAreaIndex = i;
        }
    }
    Mat dst = Mat::zeros(height_, width_, CV_8UC1);
    Scalar colour(255);
    if (contours.size())
        drawContours(dst, contours, maxAreaIndex, colour, -1, 8, hierarchy);
    table_mask_ = dst > 1;
}


int BackgroundRemovalNode::GetFlatSurfaceParams() {
    GetTableImage();
    if (show_table_mask_)
    {
        namedWindow("Table mask", WINDOW_AUTOSIZE);
        imshow("Table mask", table_mask_);
        waitKey(30);
    }
    std::vector<Point3f> xyzCoords;
    int point_count = 0;

    for (int i = 0; i < height_*width_; i++) {
        if (!std::isnan(input_pointcloud_.points[i].x) && table_mask_.at<unsigned char>(int(i/width_+0.0001),i%width_)!=0) {
            xyzCoords.push_back(Point3f(input_pointcloud_.points[i].x,input_pointcloud_.points[i].y,input_pointcloud_.points[i].z));
            point_count++;
        }
	}

    if (point_count == 0) return -1;
    Mat p = Mat(xyzCoords).reshape(1);
    Mat q = Mat::ones(point_count, 3, CV_32F);
    p.col(0).copyTo(q.col(0));
    p.col(2).copyTo(q.col(2));
    Mat params, r(point_count, 1, CV_32F);
    p.col(1).copyTo(r);
    solve(q, r, params, DECOMP_QR);
    surface_params_[0] = params.at<float>(0, 0);
    surface_params_[1] = params.at<float>(1, 0);
    surface_params_[2] = params.at<float>(2, 0);

    return 0;
}

void BackgroundRemovalNode::calculate_cropped_range() {
    if (crop_info_.left_margin == 0 && crop_info_.top_margin == 0 && crop_info_.width == 100 && crop_info_.height == 100) {
        crop_image_ = false;
    } 
    else {
        crop_image_ = true;
    }
    min_cropped_x_ = (int) (0.01 * crop_info_.left_margin * width_);
    min_cropped_y_ = (int) (0.01 * crop_info_.top_margin * height_);
    max_cropped_x_ = (int) (width_ * (0.01 * crop_info_.left_margin + 0.01 * crop_info_.width) - 1);
    max_cropped_y_ = (int) (height_ * (0.01 * crop_info_.top_margin + 0.01 * crop_info_.height) - 1);
}

bool BackgroundRemovalNode::in_cropped_frame(int index) {
    if (!crop_image_) return true;
    if (index%width_>=min_cropped_x_ && index%width_<=max_cropped_x_ && (int)(index/width_+0.0001)>=min_cropped_y_ && (int)(index/width_+0.0001)<=max_cropped_y_) return true;
    else return false;
}


/*Creates lables_: a Mat of labels, with 0 meaning 1st object,
* 1 meaning 2nd etc. Labels are given only to foreground points,
* indexed by vector foreground_ind_
*/
int BackgroundRemovalNode::ImageSegmentation() {
    calculate_cropped_range();
	if (!surface_params_found_ || recalculate_surface_params_) {
        if (GetFlatSurfaceParams() == -1) return -1;
        surface_params_found_ = true;
    }
    std::vector<Point3f> foregroundCoords;
    std::vector<int> foreground_ind;
    int foreground_pixels_num = 0;
    for (int i = 0; i < height_*width_; i++) {
        if (input_pointcloud_.points[i].y < (surface_params_[0]*input_pointcloud_.points[i].x + surface_params_[2]*input_pointcloud_.points[i].z + surface_params_[1]) - SURF_THRESH_ && input_pointcloud_.points[i].z < MAX_DEPTH_ && in_cropped_frame(i)) {
            foregroundCoords.push_back(Point3f(input_pointcloud_.points[i].x, input_pointcloud_.points[i].y, input_pointcloud_.points[i].z));
            foreground_ind.push_back(i);
            foreground_pixels_num++;
        }
    }

    Mat centers, labels;
    if (foreground_pixels_num == 0) return -1;
    if (clustering_type_ == "kmeans") {
        kmeans(foregroundCoords, NUM_OBJECTS_, labels,
               TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,10,1.0), 
               KMEANS_NUM_ATTEMPTS_, KMEANS_PP_CENTERS, centers);
    }
    else {
        clustering::DBSCAN<Eigen::VectorXf, Eigen::MatrixXf> dbscan(0.2, 20); //TODO pass params
        Mat coords_cv = Mat(foregroundCoords).reshape(1);
        Eigen::MatrixXf coords_eigen;
        cv2eigen(coords_cv, coords_eigen);
        dbscan.fit(coords_eigen);
        std::vector<int> dbscan_labels = dbscan.get_labels();
        labels = Mat(dbscan_labels).reshape(1);
    }
    labels.copyTo(labels_);
    foreground_ind_ = foreground_ind;
    return 0;
}


/*Given a set of initial pixels, creates lables_: a Mat of labels, with 0 meaning 1st object,
* 1 meaning 2nd etc. Labels are given only to foreground points,
* indexed by vector foreground_ind_
*/
int BackgroundRemovalNode::ImageSegmentation(object_assembly_msgs::Points2D points) {
    calculate_cropped_range();
	if (!surface_params_found_ || recalculate_surface_params_) {
        GetFlatSurfaceParams();
        surface_params_found_ = true;
    }
    std::vector<Point3f> foregroundCoords;
    std::vector<int> foreground_ind;
    int foreground_pixels_num = 0;

    for (int i = 0; i < height_*width_; i++) {
        if (input_pointcloud_.points[i].y < (surface_params_[0]*input_pointcloud_.points[i].x + surface_params_[2]*input_pointcloud_.points[i].z + surface_params_[1]) - SURF_THRESH_ && input_pointcloud_.points[i].z < MAX_DEPTH_ && in_cropped_frame(i)) {
            foregroundCoords.push_back(Point3f(input_pointcloud_.points[i].x, input_pointcloud_.points[i].y, input_pointcloud_.points[i].z));
            foreground_ind.push_back(i);
            foreground_pixels_num++;
        }
    }
    Mat centers(NUM_OBJECTS_,3,CV_32F), labels;
    for (int i = 0; i < points.points.size(); i++) {
        bool valid_points_found = false;
        int radius = 0;
        while (!valid_points_found) {
            int x = (int)(points.points[i].x + 0.5);
            int y = (int)(points.points[i].y + 0.5);
            Point3f mean_position(0.0,0.0,0.0);
            int valid_points = 0;
            //Check image boundaries
            for (int ix = (x-radius>0) ? x-radius : 0; ix <= ((ix+radius<width_) ? ix+radius : width_-1); ix++) {
                for (int iy = (y-radius>0) ? y-radius : 0; iy <= ((iy+radius<height_) ? iy+radius : height_-1); iy++) {
                    int index = ix + iy * width_;
                    if (depth_image_.at<unsigned long int>(iy, ix) != 0)
                    {
                        valid_points_found = true;
                        mean_position = mean_position + Point3f(
                            input_pointcloud_.points[index].x,
                            input_pointcloud_.points[index].y,
                            input_pointcloud_.points[index].z);
                        valid_points++;
                    }
                }
            }
            if (valid_points_found) {
                mean_position.x = mean_position.x/valid_points;
		        mean_position.y = mean_position.y/valid_points;
		        mean_position.z = mean_position.z/valid_points;
                centers.at<float>(i,0) = mean_position.x;
                centers.at<float>(i,1) = mean_position.y;
                centers.at<float>(i,2) = mean_position.z;
            }
            radius++;
        }
    }
    if (foreground_pixels_num == 0) return -1;
    if (clustering_type_ == "kmeans") {
        kmeans(foregroundCoords, NUM_OBJECTS_, labels,
               TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,10,1.0), 
               1, KMEANS_PP_CENTERS, centers);
    }
    else {
        clustering::DBSCAN<Eigen::VectorXf, Eigen::MatrixXf> dbscan(0.02, 20); //TODO pass params
        Mat coords_cv = Mat(foregroundCoords).reshape(1);
        Eigen::MatrixXf coords_eigen;
        cv2eigen(coords_cv, coords_eigen);
        dbscan.fit(coords_eigen);
        std::vector<int> dbscan_labels = dbscan.get_labels();
        labels = Mat(dbscan_labels).reshape(1);
    }
    labels.copyTo(labels_);
    foreground_ind_ = foreground_ind;
    return 0;
}


void BackgroundRemovalNode::createOutputPCLs() {
    output_pointclouds_ = std::vector<pcl::PointCloud<pcl::PointXYZ>>(NUM_OBJECTS_);
    pcl::PointCloud<pcl::PointXYZ> foreground_pcl;
    for (int i = 0; i < foreground_ind_.size(); i++) {
        if (labels_.at<int>(i) > -1 && labels_.at<int>(i) < NUM_OBJECTS_) {
            output_pointclouds_[labels_.at<int>(i)].push_back(input_pointcloud_.points[foreground_ind_[i]]);
            foreground_pcl.push_back(input_pointcloud_.points[foreground_ind_[i]]);
        }
    }
    foreground_pcl.header.frame_id = "kinect2_rgb_optical_frame";
    pointclouds_publisher_.publish(foreground_pcl);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pose_initializer");
    ros::NodeHandle nh("~");

    std::string depth_image_topic, point_cloud_topic;
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("point_cloud_topic", point_cloud_topic);
    float max_depth, surf_thresh;
    nh.getParam("max_depth", max_depth);
    nh.getParam("surf_threshold", surf_thresh);
    bool recalculate_surface_params;
    nh.getParam("recalculate_surface_params", recalculate_surface_params);
    std::string clustering_type;
    nh.getParam("clustering_type", clustering_type);
    if (clustering_type != "kmeans" && clustering_type != "dbscan") {
        ROS_ERROR("Invalid clustering type");
        return -1;
    }
    CropInfo crop_info;
    nh.getParam("crop_depth_image/top_margin", crop_info.top_margin);
    nh.getParam("crop_depth_image/left_margin", crop_info.left_margin);
    nh.getParam("crop_depth_image/width", crop_info.width);
    nh.getParam("crop_depth_image/height", crop_info.height);
    if (crop_info.top_margin < 0 || crop_info.left_margin < 0 || crop_info.height < 0 || crop_info.width < 0) {
        ROS_ERROR("Crop parameter negative");
        return -1;
    }
    if (crop_info.top_margin + crop_info.height > 100) {
        ROS_ERROR("Top margin + height > 100");
        return -1;
    }
    if (crop_info.left_margin + crop_info.width > 100) {
        ROS_ERROR("Left margin + width > 100");
        return -1;
    }
    bool crop_for_table_detection;
    nh.getParam("crop_for_table_detection", crop_for_table_detection);

    bool show_table_mask;
    nh.getParam("show_table_mask", show_table_mask);

    BackgroundRemovalNode background_removal_node(max_depth, surf_thresh, recalculate_surface_params, clustering_type, crop_info, crop_for_table_detection, show_table_mask);

    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, depth_image_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, point_cloud_topic, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(depth_image_sub, point_cloud_sub, 10);
    sync.registerCallback(boost::bind(
                    &BackgroundRemovalNode::callback,
                    &background_removal_node, _1, _2));

    ros::ServiceServer service_pcls_given_locations =
        nh.advertiseService(
            "FetchPCLsGivenLocations",
            &BackgroundRemovalNode::fetch_pcls_given_locations_service,
            &background_removal_node);

    ros::ServiceServer service_pcls =
        nh.advertiseService(
            "FetchPCLs",
            &BackgroundRemovalNode::fetch_pcls_service,
            &background_removal_node);

    ros::ServiceServer service_surface_params =
        nh.advertiseService(
            "FetchSurfaceParams",
            &BackgroundRemovalNode::fetch_surface_params_service,
            &background_removal_node);

    ros::spin();
    return 0;
}
