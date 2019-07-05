#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <vector> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>

using namespace cv;

class ImageCropNode{

public:
    ImageCropNode(int crop_height,
                  int crop_width,
                  int crop_top,
                  int crop_left,
                  double scaling_factor,
                  std::string new_type,
                  bool remove_hand,
                  std::vector<int> hsv_range);
    void CameraInfoCallback(const sensor_msgs::CameraInfo &cinfo);
    void depth_callback(const sensor_msgs::ImageConstPtr& image);
    void colour_callback(const sensor_msgs::ImageConstPtr& image);

private:
    int height_;
    int width_;
    bool image_received_ = false;
    bool cinfo_set_ = false;
    int crop_height_;
    int crop_width_;
    int crop_top_;
    int crop_left_;
    int sd_crop_height_;
    int sd_crop_width_;
    int sd_crop_top_;
    int sd_crop_left_;
    double scaling_factor_;
    std::string new_type_;
    const std::vector<int> hsv_range_;
    bool remove_hand_;
    Mat hand_mask_;
    bool hand_mask_available_ = false;
    ros::NodeHandle node_handle_;
    ros::Publisher cropped_image_publisher_;
    ros::Publisher newcamerainfo_publisher_;

    sensor_msgs::CameraInfo cinfo_;
};

ImageCropNode::ImageCropNode(int crop_height,
                             int crop_width,
                             int crop_top,
                             int crop_left,
                             double scaling_factor,
                             std::string new_type,
                             bool remove_hand,
                             std::vector<int> hsv_range) : node_handle_("~"),
                                                     crop_height_(crop_height),
                                                     crop_width_(crop_width),
                                                     crop_top_(crop_top),
                                                     crop_left_(crop_left),
                                                     scaling_factor_(scaling_factor),
                                                     new_type_(new_type),
                                                     remove_hand_(remove_hand),
                                                     hsv_range_(hsv_range)
{
    sd_crop_height_ = (int)(((double)crop_height_)/2.547);
    sd_crop_width_ = (int)(((double)crop_width_)/3.75);
    sd_crop_top_ = (int)(((double)crop_top_)/2.547);
    sd_crop_left_ = (int)(((double)crop_left_)/3.75);
    cropped_image_publisher_ = node_handle_.advertise<sensor_msgs::Image>("cropped_image", 0);
    newcamerainfo_publisher_ = node_handle_.advertise<sensor_msgs::CameraInfo>("newCameraInfo", 0);
}

void ImageCropNode::CameraInfoCallback(const sensor_msgs::CameraInfo &cinfo) {
    if (image_received_) {
        if (!cinfo_set_) {
            cinfo_ = cinfo;
            cinfo_.P[2] = width_/2 - crop_left_ - 0.5;
            cinfo_.P[6] = height_/2 - crop_top_ - 0.5;
            cinfo_.K[2] = cinfo_.P[2];
            cinfo_.K[5] = cinfo_.P[6];
            cinfo_.height = crop_height_;
            cinfo_.width = crop_width_;
            cinfo_set_ = true;
        }
        newcamerainfo_publisher_.publish(cinfo_);
    }
}

void ImageCropNode::depth_callback(
        const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.",
                  image->encoding.c_str());
    }
    image_received_ = true;
    height_ = cv_ptr->image.rows;
    width_ = cv_ptr->image.cols;
    if (crop_left_+crop_width_ > width_ || crop_top_+crop_height_ > height_)
        ROS_ERROR("Invalid crop parameters");
    Rect r1(crop_left_, crop_top_, crop_width_, crop_height_);
    Mat img = cv_ptr->image(r1);

    cv_ptr->image = img;
    //img.copyTo(cv_ptr->image);
    cv_ptr->image.convertTo(cv_ptr->image, CV_32FC1, scaling_factor_);
    //std::cout << "before: " << cv_ptr->image.at<float>(100, 100) << "\n";
    //Mat in_range = img < 1000.0;
    //Mat in_range_32FC1;
    //in_range.convertTo(in_range_32FC1, CV_32FC1);
    //normalize(in_range_32FC1, in_range_32FC1, 0, 1., NORM_MINMAX);
    if (remove_hand_ && hand_mask_available_)
    {
        Mat temp;
        cv_ptr->image.copyTo(temp, hand_mask_);
        cv_ptr->image = cv_ptr->image - temp;
    }
    ////cv_ptr->image = cv_ptr->image * in_range_32FC1;
    //std::cout << "after: " << cv_ptr->image.at<float>(100, 100) << "\n";
    auto img_msg = cv_ptr->toImageMsg();
    img_msg->encoding = "32FC1";

    //double depth_min, depth_max;
    //cv::minMaxLoc(cv_ptr->image, &depth_min, &depth_max);
    //std::cout << "MIN: " << depth_min << " MAX: " << depth_max << "\n";

    cropped_image_publisher_.publish(img_msg);
}

void ImageCropNode::colour_callback(const sensor_msgs::ImageConstPtr& image)
{
    Mat cameraFeed;
    try {
        (cv_bridge::toCvShare(image,"bgr8")->image).copyTo(cameraFeed);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
    }
    Rect r1(sd_crop_left_, sd_crop_top_, sd_crop_width_, sd_crop_height_);
    cameraFeed = cameraFeed(r1);
    Mat HSV;
    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
    if (hsv_range_[0] < hsv_range_[1])
        inRange(HSV, Scalar(hsv_range_[0], hsv_range_[2], hsv_range_[4]), Scalar(hsv_range_[1], hsv_range_[3], hsv_range_[5]), hand_mask_);
    else
    {
        Mat mask1, mask2;
        inRange(HSV, Scalar(0, hsv_range_[2], hsv_range_[4]), Scalar(hsv_range_[1], hsv_range_[3], hsv_range_[5]), mask1);
        inRange(HSV, Scalar(hsv_range_[0], hsv_range_[2], hsv_range_[4]), Scalar(180, hsv_range_[3], hsv_range_[5]), mask2);
        bitwise_or(mask1, mask2, hand_mask_);
    }
    resize(hand_mask_, hand_mask_, Size(crop_width_, crop_height_));
    hand_mask_available_ = true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_crop");
    ros::NodeHandle nh("~");

    std::string depth_image_topic;
    nh.getParam("depth_image_topic", depth_image_topic);
    std::string colour_image_topic;
    nh.getParam("colour_image_topic", colour_image_topic);
    std::string camera_info_topic;
    nh.getParam("camera_info_topic", camera_info_topic);

    std::string new_type;;
    nh.getParam("convert_to", new_type);

    int crop_height, crop_width, crop_top, crop_left;
    double scaling_factor;
    nh.getParam("crop_height", crop_height);
    nh.getParam("crop_width", crop_width);
    nh.getParam("crop_top", crop_top);
    nh.getParam("crop_left", crop_left);
    nh.getParam("scaling_factor", scaling_factor);
    bool remove_hand;
    nh.getParam("remove_hand", remove_hand);
    std::vector<int> hsv_range;
    nh.getParam("hsv_range", hsv_range);

    if (crop_left < 0 || crop_width < 0 || crop_top < 0 || crop_height < 0 || scaling_factor < 0)
        ROS_ERROR("Negative crop parameters");

    ImageCropNode image_crop_node(crop_height, crop_width, crop_top, crop_left, scaling_factor, new_type, remove_hand, hsv_range);

    ros::Subscriber depth_image_subscriber = 
        nh.subscribe(depth_image_topic,
                     1,
                     &ImageCropNode::depth_callback,
                     &image_crop_node);

    ros::Subscriber colour_image_subscriber;
    if (remove_hand)
    {
        colour_image_subscriber = 
            nh.subscribe(colour_image_topic,
                         1,
                         &ImageCropNode::colour_callback,
                         &image_crop_node);
    }

    ros::Subscriber cinfosubscriber =
        nh.subscribe(camera_info_topic,
                     1,
                     &ImageCropNode::CameraInfoCallback,
                     &image_crop_node);

    ros::spin();
}
