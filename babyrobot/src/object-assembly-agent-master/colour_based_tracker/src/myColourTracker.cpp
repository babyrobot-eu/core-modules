//myColourTracker.cpp

//Original code written by  Kyle Hounslow 2013
//Modified by Jack Hadfield 2016

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.


#include "myColourTracker.hpp"

ThingToFind::ThingToFind(std::vector<int> val, std::string strname)
{
	for (int i=0;i<6;i++)
		values[i] = val[i];
	name = strname;
}

ThingToFind::~ThingToFind()
{
}

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 180;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;


std::string intToString(int number) {
    	std::stringstream ss;
    	ss << number;
    	return ss.str();
}

void drawObject(int x, int y, Mat &frame, std::string objectName) {
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    Scalar colour;

    if (objectName=="Green Box")
        colour = Scalar(0, 192, 0);
    if (objectName=="Red Box")
        colour = Scalar(0, 0, 255);
    if (objectName=="Blue Box")
        colour = Scalar(255, 128, 0);
    if (objectName=="Yellow Box")
        colour = Scalar(0, 255, 255);
    if (objectName=="Purple Box")
        colour = Scalar(255, 0, 128);

	circle(frame, Point(x, y), 20, colour, 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), colour, 2);
	else line(frame, Point(x, y), Point(x, 0), colour, 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), colour, 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), colour, 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), colour, 2);
	else line(frame, Point(x, y), Point(0, y), colour, 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), colour, 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), colour, 2);

	putText(frame, objectName + intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, colour, 2);

}

void drawObject(int x, int y, Mat &frame, std::string objectName, Scalar colour) {
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	circle(frame, Point(x, y), 20, colour, 2);

	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), colour, 2);
	else line(frame, Point(x, y), Point(x, 0), colour, 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), colour, 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), colour, 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), colour, 2);
	else line(frame, Point(x, y), Point(0, y), colour, 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), colour, 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), colour, 2);

	putText(frame, objectName, Point(x, y + 30), 1, 1, colour, 2);// + intToString(x) + "," + intToString(y)
}

void drawObject(int x, int y, Mat &frame, std::string objectName, Scalar colour, int frame_height, int frame_width) {
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	circle(frame, Point(x, y), 20, colour, 2);

	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), colour, 2);
	else line(frame, Point(x, y), Point(x, 0), colour, 2);
	if (y + 25<frame_height)
		line(frame, Point(x, y), Point(x, y + 25), colour, 2);
	else line(frame, Point(x, y), Point(x, frame_height), colour, 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), colour, 2);
	else line(frame, Point(x, y), Point(0, y), colour, 2);
	if (x + 25<frame_width)
		line(frame, Point(x, y), Point(x + 25, y), colour, 2);
	else line(frame, Point(x, y), Point(frame_width, y), colour, 2);

	putText(frame, objectName + intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, colour, 2);
}

void mouse_callback(int  event, int  x, int  y, int  flag, void *param) {
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        Point pt;
        pt.x = x;
        pt.y = y;
        CLICKED_POINTS.push_back(pt);
        //NEWCOORDS = true;
    }
}

ColourTrackerNode::ColourTrackerNode(
            std::vector<std::string> object_names,
            std::vector<std::vector<int>> hsv_ranges,
            bool flags[6],
            int tracker_params[5],
            std::vector<int> crop_range,
            std::vector<double> resize_coeffs,
            int particle_filter_downsampling,
            double std_coeff,
            sensor_msgs::CameraInfo rgb_cinfo,
            sensor_msgs::CameraInfo depth_cinfo) : 
                    node_handle_("~"),
                    object_names_(object_names),
                    crop_range_(crop_range),
                    resize_coeffs_(resize_coeffs),
                    particle_filter_downsampling_
                        (particle_filter_downsampling),
                    num_objects_(object_names.size()),
                    std_coeff_(std_coeff)
{
    std::cout << "Creating Colour Based Tracker Node\n";
    for (int i = 0; i < hsv_ranges.size(); i++) {
        ThingToFind box(hsv_ranges[i], object_names[i]);
        boxes_.push_back(box);
        Mat temp_colour(1, 1, CV_8UC3);
        temp_colour.at<Vec3b>(Point(0,0)) = Vec3b((hsv_ranges[i][0]+hsv_ranges[i][1])/2, (hsv_ranges[i][2]+hsv_ranges[i][3])/2, (hsv_ranges[i][4]+hsv_ranges[i][5])/2);
        cvtColor(temp_colour, temp_colour, COLOR_HSV2BGR);
        colours_.push_back(Scalar(temp_colour.at<Vec3b>(Point(0,0))[0],temp_colour.at<Vec3b>(Point(0,0))[1],temp_colour.at<Vec3b>(Point(0,0))[2]));
    }

    std::vector<int> temp;
    //store some zeros
    for (int j = 0; j < pastValues_; j++)
        temp.push_back(0);
    for (int i = 0; i < num_objects_; i++) {
        x_.push_back(0);
        y_.push_back(0);
        past_x_.push_back(temp);
        past_y_.push_back(temp);
        x_estimate_.push_back(0);
        y_estimate_.push_back(0);
        scores_.push_back(0);
    }
    show_RGB_ = flags[0];
    show_HSV_ = flags[1];
    show_threshold_ = flags[2];
    useTrackbars_ = flags[3];
    smooth_estimate_ = flags[4];
    hsv_ranges_available_ = !flags[5];
    MAX_NUM_OBJECTS_ = tracker_params[0];
    MIN_OBJECT_AREA_ = tracker_params[1];
    MAX_OBJECT_AREA_ = tracker_params[2];
    erode_size_ = tracker_params[3];
    dilate_size_ = tracker_params[4];

    //create slider bars for HSV filtering
    if (useTrackbars_) 
        createTrackbars();

    position_estimates_publisher_ = node_handle_.advertise<object_assembly_msgs::Points2D>("objects_Points2D", 0);
    input_publisher_ = node_handle_.advertise<object_assembly_msgs::Input>("input", 0);
    hsv_ranges_publisher_ = node_handle_.advertise<object_assembly_msgs::HSVRanges>("HSV_ranges", 0);

	if (show_RGB_)
        namedWindow(windowName, WINDOW_AUTOSIZE);
    if (!hsv_ranges_available_)
        selector_ = new Selector(windowName.c_str());

    //Colour to depth matching parameters
    x_focal_ratio_ = depth_cinfo.P[0] / rgb_cinfo.P[0];
    y_focal_ratio_ = depth_cinfo.P[5] / rgb_cinfo.P[5];
    x_offset_ = depth_cinfo.P[2] - (rgb_cinfo.P[2]-crop_range_[0]) * x_focal_ratio_;
    y_offset_ = depth_cinfo.P[6] - (rgb_cinfo.P[6]-crop_range_[1]) * y_focal_ratio_;
}


void ColourTrackerNode::colour_tracker_callback(const sensor_msgs::ImageConstPtr& ros_image)
{ 
    Mat cameraFeed;
    try {
        (cv_bridge::toCvShare(ros_image,"bgr8")->image).copyTo(cameraFeed);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", ros_image->encoding.c_str());
    }
    Rect r1(crop_range_[0], crop_range_[1], crop_range_[2], crop_range_[3]);
    cameraFeed = cameraFeed(r1);
    doStuff(cameraFeed);
}


void ColourTrackerNode::depth_tracker_callback(const sensor_msgs::ImageConstPtr& ros_image)
{
    std::lock_guard<std::mutex> lock_input(depth_mutex_);
    try {
        (cv_bridge::toCvShare(ros_image,"")->image).copyTo(depth_image_);
        depth_available_ = true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", ros_image->encoding.c_str());
    }
    //std::cout << "MPLUUUUUU: " << depth_image_.at<float>((int)(50*2.964-58.6699-12),(int)(10*2.964+92.785+80)) << "\n";
    //int pt = ((int)(10*2.964+92.785+80) + ros_image->width * (int)(50*2.964-58.6699-12));
    //float *p = (float*)(ros_image->data.data()+4*pt);
    //std::cout << "MPLEEEEEE: " << *p << "\n";
}


void ColourTrackerNode::publish() {
    object_assembly_msgs::Points2D points;
    if (CLICKED_POINTS.size() < num_objects_) {
        for (int i = 0; i < x_.size(); i++) {
            object_assembly_msgs::Point2D point;
            point.x = x_estimate_[i];
            point.y = y_estimate_[i];
            point.name = boxes_[i].name;
            points.points.push_back(point);
        }
    }
    else {
        for (int i = 0; i < x_.size(); i++) {
            object_assembly_msgs::Point2D point;
            point.x = CLICKED_POINTS[i].x;
            point.y = CLICKED_POINTS[i].y;
            point.name = boxes_[i].name;
            points.points.push_back(point);
        }
    }
    position_estimates_publisher_.publish(points);
    
    object_assembly_msgs::Input input;
    for (int i = 0; i < num_objects_; i++) {
        //input.input.push_back(resize_coeffs_[0]*points.points[i].x/particle_filter_downsampling_);
        //input.input.push_back(resize_coeffs_[1]*points.points[i].y/particle_filter_downsampling_);
        input.input.push_back((points.points[i].x*x_focal_ratio_+x_offset_)/particle_filter_downsampling_);
        input.input.push_back((points.points[i].y*y_focal_ratio_+y_offset_)/particle_filter_downsampling_);
        input.input.push_back(0.8); //TODO remove this line
    }
    for (int i = 0; i < num_objects_; i++) {
        input.input.push_back(scores_[i]);
    }
    input_publisher_.publish(input);
    object_assembly_msgs::HSVRanges ranges_msg;
    for (int i = 0; i < num_objects_; i++)
        for (int j = 0; j < 6; j++)
            ranges_msg.ranges.push_back(boxes_[i].values[j]);
    hsv_ranges_publisher_.publish(ranges_msg);
}


void ColourTrackerNode::on_trackbar(int, void*) {
    //This function gets called whenever a
    // trackbar position is changed
}


void ColourTrackerNode::createTrackbars() {
    //create window for trackbars

    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH), 
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->      
    createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);

}


void ColourTrackerNode::morphOps(Mat &thresh) {
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(erode_size_, erode_size_));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(dilate_size_, dilate_size_));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}


double ColourTrackerNode::smoothEstimate(std::vector<int> &past_values,
                          int current_value) {
//TODO smooth based only on previous estimate
    double estimate = 0.0312*current_value; //extra to add up to 1
    past_values.erase(past_values.begin(), past_values.begin()+1);
    past_values.push_back(current_value);
    for (int k=past_values.size()-1;k>=0;k--) {
        estimate = estimate + past_values[k]/pow(2,k+1);
    }
    return estimate;
}


bool ColourTrackerNode::trackFilteredObject(
            int &x,
            int &y,
            Mat threshold,
            Mat &cameraFeed,
            std::string objectName,
            double &score)
{
    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    double totalArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
    	int numObjects = hierarchy.size();
    	//if number of objects greater than MAX_NUM_OBJECTS_ we have a noisy filter
        if (numObjects<MAX_NUM_OBJECTS_) {
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;
                totalArea += area;                

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if (area>MIN_OBJECT_AREA_ && area<MAX_OBJECT_AREA_ && area>refArea) {
                    x = moment.m10 / area;
                    y = moment.m01 / area;
                    objectFound = true;
                    refArea = area;
                }
                else ;//objectFound = false;

            }
    /*
            //let user know you found an object
            if (objectFound == true) {
                putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                //draw object location on screen
                drawObject(x, y, cameraFeed, objectName);
            }
    */

        }
        else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
    if (totalArea != 0)
        score = refArea/totalArea;
    else
        score = 0;
    return objectFound;
}


void ColourTrackerNode::doStuff(Mat &cameraFeed) {
    //convert frame from BGR to HSV colorspace
    cvtColor(cameraFeed, HSV_, COLOR_BGR2HSV);
        //setMouseCallback(windowName, mouse_callback);
    if (hsv_ranges_available_)
    {
        if (depth_available_)
        {
            std::lock_guard<std::mutex> lock_input(depth_mutex_);
            Mat in[] = {depth_image_, depth_image_, depth_image_};
            merge(in, 3, depth_3c_);
        }
        //repeat for all objects
        for (int obj = 0; obj < num_objects_; obj++) {
            for (int i = 0; i < 6; i++)
		        minmax_[i] = boxes_[obj].values[i];
		    objectName_ = boxes_[obj].name;
            //filter HSV image between values and store filtered 
		    //image to threshold matrix
            if (useTrackbars_)
                inRange(HSV_, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold_);
            else
                inRange(HSV_, Scalar(minmax_[0], minmax_[2], minmax_[4]), Scalar(minmax_[1], minmax_[3], minmax_[5]), threshold_);
                
            if (useMorphOps_)
                morphOps(threshold_);

            if (trackFilteredObject(x_[obj], y_[obj], threshold_, cameraFeed, objectName_, scores_[obj])) {
            //let user know an object was found
                if (smooth_estimate_) {
                    x_estimate_[obj] = smoothEstimate(past_x_[obj], x_[obj]);
                    y_estimate_[obj] = smoothEstimate(past_y_[obj], y_[obj]);
                }
                else {
                    x_estimate_[obj] = (double)x_[obj];
                    y_estimate_[obj] = (double)y_[obj];
                }
                //draw object location on screen
                //drawObject((int)x_estimate_[obj], (int)y_estimate_[obj], cameraFeed, objectName_);
                drawObject((int)x_estimate_[obj], (int)y_estimate_[obj], cameraFeed, objectName_, colours_[obj]);
                if (depth_available_)
                {
                    //std::cout << "MPLAAAAAAA: " << obj << " " << depth_image_.at<float>((int)(y_estimate_[obj]*2.964-58.6699-12),(int)(x_estimate_[obj]*2.964+92.785+80)) << "\n"; 
                    drawObject((int)(x_estimate_[obj]*x_focal_ratio_+x_offset_), (int)(y_estimate_[obj]*y_focal_ratio_+y_offset_), depth_3c_, objectName_, colours_[obj], 540, 1700);
                }
            }
        }

        publish();

        if (depth_available_)
        {
            imshow("Depth Image", depth_3c_);
        }


    }
    else
    {
        Mat rgb[3];
        split(cameraFeed, rgb);
        Mat mask = rgb[0] != 0;
        bitwise_or(rgb[1]!=0, mask, mask);
        bitwise_or(rgb[2]!=0, mask, mask);
        //imshow("maskkk", mask);
        if (selector_->calculate_ranges(HSV_, means_, stds_, mask) >= num_objects_)
        {
            hsv_ranges_available_ = true;
            for (int obj = 0; obj < num_objects_; obj++)
            {
                ThingToFind box(std::vector<int>(6,0), object_names_[obj]);
                box.values[0] = (int)(means_[obj].at<double>(0,0) - std_coeff_ * stds_[obj].at<double>(0,0));
                box.values[2] = (int)(means_[obj].at<double>(1,0) - std_coeff_ * stds_[obj].at<double>(1,0));
                box.values[4] = (int)(means_[obj].at<double>(2,0) - std_coeff_ * stds_[obj].at<double>(2,0));
                box.values[1] = (int)(means_[obj].at<double>(0,0) + std_coeff_ * stds_[obj].at<double>(0,0));
                box.values[3] = (int)(means_[obj].at<double>(1,0) + std_coeff_ * stds_[obj].at<double>(1,0));
                box.values[5] = (int)(means_[obj].at<double>(2,0) + std_coeff_ * stds_[obj].at<double>(2,0));
                boxes_.push_back(box);
std::cout << "VAL: " << boxes_[obj].values[0] << " to " << boxes_[obj].values[1] << "\n";
                Mat temp_colour(1, 1, CV_8UC3);
                temp_colour.at<Vec3b>(Point(0,0)) = Vec3b((minmax_[0]+minmax_[1])/2, (minmax_[2]+minmax_[3])/2, (minmax_[4]+minmax_[5])/2);
                cvtColor(temp_colour, temp_colour, COLOR_HSV2BGR);
                colours_.push_back(Scalar(temp_colour.at<Vec3b>(Point(0,0))[0],temp_colour.at<Vec3b>(Point(0,0))[1],temp_colour.at<Vec3b>(Point(0,0))[2]));
            }
        }
    }

    if (show_threshold_)
        imshow(windowName2, threshold_);
    if (show_RGB_)
        imshow(windowName, cameraFeed);
    if (show_HSV_)
        imshow(windowName1, HSV_);

    waitKey(30);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "colour_based_tracker");
    ros::NodeHandle nh("~");

    //Read params from config file
    int num_objects;
    nh.getParam("number_of_objects", num_objects);

    std::string camera_topic;
    nh.getParam("camera_topic", camera_topic);
    bool flags[6];
    nh.getParam("show_rgb", flags[0]);
    nh.getParam("show_hsv", flags[1]);
    nh.getParam("show_threshold", flags[2]);
    nh.getParam("use_trackbars", flags[3]);
    nh.getParam("smooth_estimate", flags[4]);
    nh.getParam("interactive_hsv_ranges", flags[5]);

    std::vector<std::string> object_names;
    std::vector<std::vector<int>> hsv_ranges;
    for (int i = 0; i < num_objects; i++) {
        std::string object_pre = std::string("objects/") + "object" + std::to_string(i+1) + "/";
        std::string object_name;
        nh.getParam(object_pre + "name", object_name);
        object_names.push_back(object_name);
        if (!flags[5])
        {
            std::vector<int> hsv_range;
            nh.getParam(object_pre + "hsv_range", hsv_range);
            hsv_ranges.push_back(hsv_range); 
        }       
    }

    int tracker_params[5];
    nh.getParam("MAX_NUM_OBJECTS", tracker_params[0]);
    nh.getParam("MIN_OBJECT_AREA", tracker_params[1]);
    nh.getParam("MAX_OBJECT_AREA", tracker_params[2]);
    nh.getParam("erode_size", tracker_params[3]);
    nh.getParam("dilate_size", tracker_params[4]);

    std::vector<int> crop_range;
    nh.getParam("crop_range", crop_range);
    std::vector<double> resize_coeffs;
    nh.getParam("resize_coefficients", resize_coeffs);
    if (resize_coeffs.size() != 2)
    {
        ROS_ERROR("resize_coefficients must have two values");
        return -1;
    }
    int particle_filter_downsampling;
    nh.getParam("particle_filter_downsampling", particle_filter_downsampling);
    double std_coeff = 1.0;
    nh.getParam("std_coefficient", std_coeff);

    std::string rgb_camera_info_topic, depth_camera_info_topic;
    nh.getParam("rgb_camera_info_topic", rgb_camera_info_topic);
    nh.getParam("depth_camera_info_topic", depth_camera_info_topic);
std::cout << "waiting for topic: " << rgb_camera_info_topic << "\n";
    sensor_msgs::CameraInfo rgb_cinfo = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rgb_camera_info_topic);
std::cout << "waiting for topic: " << depth_camera_info_topic << "\n";
    sensor_msgs::CameraInfo depth_cinfo = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(depth_camera_info_topic);
std::cout << "Got topics\n";

    ColourTrackerNode colour_tracker_node(object_names, hsv_ranges, flags, tracker_params, crop_range, resize_coeffs, particle_filter_downsampling, std_coeff, rgb_cinfo, depth_cinfo);

    ros::Subscriber subscriber = nh.subscribe(camera_topic, 1, &ColourTrackerNode::colour_tracker_callback, &colour_tracker_node);

    ros::Subscriber depth_subscriber = nh.subscribe("/image_crop/cropped_imageINVALID", 1, &ColourTrackerNode::depth_tracker_callback, &colour_tracker_node);

    ros::spin();
}
