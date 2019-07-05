#ifndef INITIALIZE_H_
#define INITIALIZE_H_

#include "DenseTrack.h"
#include "ros/ros.h"
//#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <deque>
#include <vector>
#include <algorithm>
#include <ctime>
enum COMMAND {IGNORE, PROCESS, CLASSIFY, REJECT};
std::string command_names[] = {"ignore", "process", "classify", "reject"};

using namespace cv;

void mySigintHandler(int sig)
{
    std::cout << "Goodbye!" << std::endl;
    destroyAllWindows();
    ros::shutdown();
}

class frame_manager {
public:
    ros::Subscriber image_sub, segmentation_sub, person_sub;
    std::deque<Mat> cache_frame_buffer;
    std::deque<Mat> frame_buffer;
    std::deque<int64_t> cache_timestamp_buffer;
    std::deque<int64_t> timestamp_buffer;
    int frame_counter, command, previous_command, segm_counter, incoming_command_id, show_track, skip_frames, process_frames_cnt;
	int x,y,h,w;
    int64 timestamp, start_time, end_time;
    bool start, cache_frames;
    int buffer_max_length;
    std::fstream * logfile_fs;
    std::string kinect_topic, activity_det_topic, person_det_topic;
    
    frame_manager(ros::NodeHandle node_handle, std::fstream * _logfile_fs, std::string _kinect_topic, std::string _activity_det_topic, std::string _person_det_topic): kinect_topic(_kinect_topic), activity_det_topic(_activity_det_topic), person_det_topic(_person_det_topic) {
	//frame_manager(ros::NodeHandle node_handle, std::fstream * _logfile_fs, std::string _kinect_topic, std::string _activity_det_topic): kinect_topic(_kinect_topic), activity_det_topic(_activity_det_topic) {
    
	    image_sub = node_handle.subscribe(kinect_topic, 10, &frame_manager::kinect_callback, this);
        segmentation_sub = node_handle.subscribe(activity_det_topic, 10, &frame_manager::segmentation_callback, this);
		person_sub = node_handle.subscribe(person_det_topic, 10, &frame_manager::person_callback, this);
        logfile_fs = _logfile_fs;
        show_track = 0;
        skip_frames = 2;
        frame_counter = 0;
        process_frames_cnt = 0;
        segm_counter = 0;
        command = IGNORE;
        previous_command = IGNORE;
        start = true;
        cache_frames = true;
        buffer_max_length=500;
        std::cout << "Hello! My name is 'frame_manager'." << std::endl;
        std::cout << "Listening to " << this->kinect_topic << " and " << this->activity_det_topic << " and " << this->person_det_topic << std::endl;
        //std::cout << "Listening to " << this->kinect_topic << " and " << this->activity_det_topic << std::endl;
    
		if(show_track == 1)
            namedWindow("Incoming Image", 0);
    }
    
    void kinect_callback(const sensor_msgs::CompressedImage::ConstPtr &image_msg) {
        Mat frame;
//        std::cout << "Entering Kinect callback " << frame_counter << std::endl;
        frame_counter++;
        if (this->command == PROCESS)
            process_frames_cnt++;
        if ((skip_frames>1) && (frame_counter%skip_frames!=0))
            return;
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_ptr->image.copyTo(frame);
        
        if (this->cache_frames) {
            this->cache_frame_buffer.push_front(frame);
            this->cache_timestamp_buffer.push_front((image_msg->header.stamp).toNSec());
            if (this->cache_frame_buffer.size()>this->buffer_max_length) {
                this->cache_frame_buffer.pop_back();
                this->cache_timestamp_buffer.pop_back();
            }
        }
        
        if (this->command==PROCESS) {
            this->frame_buffer.push_front(frame);
            this->timestamp_buffer.push_front((image_msg->header.stamp).toNSec());
        }
        
        if(show_track == 1) {
            imshow( "Incoming Image", frame);
            cvWaitKey(25);
        }
    };
    
    
//    Mat getFrame() {
//        Mat frame;
//        this->cache_frame_buffer.front().copyTo(frame);
////        this->cache_frame_buffer.pop_front();
//        return frame;
//    }
    
	void person_callback(const std_msgs::String::ConstPtr &segm_msg) {
		//std::cout << "Entering Person callback " << std::endl;
		//std::cout << "I just heard " << segm_msg->data << std::endl;
		int i;
        std::stringstream tmp(segm_msg->data);
		tmp >> i;
        tmp >> this->x;
        tmp >> this->y;
		tmp >> this->w;
		tmp >> this->h;
		//std::cout << this->x << this->y << this->h << this->w << std::endl;
	}
	
    void segmentation_callback(const std_msgs::String::ConstPtr &segm_msg) {
//        std::cout << "Entering Segmentation callback " << segm_counter++ << std::endl;
//        std::cout << "I just heard " << segm_msg->data << std::endl;
        std::stringstream tmp(segm_msg->data);
        tmp >> this->incoming_command_id;
        tmp >>  this->timestamp;
        if (this->incoming_command_id==100) {
            std::cout << "Goodbye!" << std::endl;
            (*logfile_fs).close();
            destroyAllWindows();
            ros::shutdown();
            return;
        }
//        std::cout << "I just heard " << incoming_command_id << " " << timestamp << std::endl;
        
        if (this->command==CLASSIFY) {
//            std::cout << "Please wait until the previous segment is classified. Sorry for the inconvenience. Ignoring the message..." << std:: endl;
            return;
        }
        if (this->incoming_command_id==0) { //start processing
            if (this->command==PROCESS)
//                    std::cout << "You sent me the same command twice. Ignoring the message..." << std:: endl;
                return;
            else {
                this->command = PROCESS;
                if (this->cache_frames) {
                   this->prepare_buffer();
                }
                this->start_time = this->timestamp;
            }
        }
        else if (this->incoming_command_id==1)  { // stop processing and classify
            if (this->command!=PROCESS)
//                std::cout << "You cannot send me '" << command_names[this->command] << "' followed by 'classify'. Ignoring the message..." << std:: endl;
                return;
            else {
                this->command = CLASSIFY;
                this->check_buffer();
                this->end_time = this->timestamp;
            }
        }
        else if (this->incoming_command_id==2) {   // reject current processing
            if (this->command!=PROCESS) {
//                std::cout << "You cannot send me '" << command_names[this->command] << "' followed by 'reject'. Ignoring the message..." << std:: endl;
            }
            else {
                this->command = REJECT;
            }
        };
//        std::cout << "The current command is:  " << command_names[this->command] << std::endl;
    }
    
    void prepare_buffer() {
//        std::cout << "cache size: " << this->cache_frame_buffer.size() << " buffer size: " << this->frame_buffer.size() << std::endl;
        std::deque<int64>::reverse_iterator i;
        for (i = this->cache_timestamp_buffer.rbegin(); i != cache_timestamp_buffer.rend(); i++) {
            if (((*i) - this->timestamp) >= 0) {
//                std::cout << "Comparing frame: " << *i << " with " << this->timestamp << " (" << (*i) - this->timestamp << ") " << std::endl;
//                std::cout << "Appended" << std::endl;
                frame_buffer.push_front(cache_frame_buffer.back());
                timestamp_buffer.push_front(cache_timestamp_buffer.back());
            }
            cache_frame_buffer.pop_back();
            cache_timestamp_buffer.pop_back();
        }
//        std::cout << "cache size: " << this->cache_frame_buffer.size() << " buffer size: " << this->frame_buffer.size() << std::endl;
    };
    
    void check_buffer() {
//        std::cout << " buffer size: " << this->frame_buffer.size() << std::endl;
        std::deque<int64>::iterator i;
        for (i = this->timestamp_buffer.begin(); i != timestamp_buffer.end(); i++) {
            if (((*i) - this->timestamp) >= 0) {
//                std::cout << "Comparing frame: " << *i << " with " << this->timestamp << " (" << (*i) - this->timestamp << ") " << std::endl;
//                std::cout << "Deleted" << std::endl;
                frame_buffer.pop_front();
                timestamp_buffer.pop_front();
            }
        }
//        std::cout << " buffer size: " << this->frame_buffer.size() << std::endl;
    };
    
    void acknowledge(int c) {
        if (c!=this->command)
            std::cout << "Synchronization error. The last messages will be ignored." << std:: endl;
        if ((c==REJECT)||(c==CLASSIFY)) {
            this->frame_buffer.clear();
            this->timestamp_buffer.clear();
            this->command = IGNORE;
            this->start = true;
            this->process_frames_cnt = 0;
        };
    }

};

class fusion_link {
public:
    ros::Publisher output_pub;
    std::vector<std::string> labels;
    int num_classes;
    std_msgs::String msg;
    std::string output_topic;
    int h1, h2, bm_idx;
    fusion_link(ros::NodeHandle node_handle, std::vector<std::string>  _labels, int _num_classes, std::string _output_topic) : labels(_labels), num_classes(_num_classes), output_topic(_output_topic) {
        std::cout << "Hello! My name is 'fusion_link'." << std::endl;
        std::cout << "Advertising to " << this->output_topic << std::endl;
//        int i;
        bm_idx = -1;
//        output_topic = "spk_action_rec";
        output_pub = node_handle.advertise<std_msgs::String>(output_topic, 1000);
//        std::cout << "Available classes (" << num_classes << "):" << std::endl;
//        for (i=0; i<num_classes; i++){
//            std::cout  << labels[i] << " ";
//            if (labels[i]=="BM") {
//                bm_idx = i;
//            }
//        }
//        std::cout << std::endl;
//        std::cout << "BM index: " << bm_idx << std::endl;
    }
    
    
    int broadcast_result(double *probs, int64 start_time, int64 end_time){
        int i;
        std::stringstream ss;
        std::vector<double> probs_;
        
        if (bm_idx>=0)
            probs[bm_idx] = -5;
        

        ss << "1 " << start_time << " " << end_time << " ";

		//for (i=0; (i<num_classes) && (i!=bm_idx); i++){
        for (i=0; (i<=num_classes-1); i++){
            ss << labels[i] << " " << probs[i] << " ";
        }
        msg.data = ss.str();
        output_pub.publish(msg);
		//std::cout << " I just published: " << ss.str() << std::endl;
        return 0;
    }
    
};

const std::string currentDateTime() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,100,"%d_%m_%Y_%I_%M_%S",timeinfo);
    std::string str(buffer);
    return str;
}

void InitTrackInfo(TrackInfo* trackInfo, int track_length, int init_gap)
{
	trackInfo->length = track_length;
	trackInfo->gap = init_gap;
}

DescMat* InitDescMat(int height, int width, int nBins)
{
	DescMat* descMat = (DescMat*)malloc(sizeof(DescMat));
	descMat->height = height;
	descMat->width = width;
	descMat->nBins = nBins;

	long size = height*width*nBins;
	descMat->desc = (float*)malloc(size*sizeof(float));
	memset(descMat->desc, 0, size*sizeof(float));
	return descMat;
}

void ReleDescMat(DescMat* descMat)
{
	free(descMat->desc);
	free(descMat);
}

void InitDescInfo(DescInfo* descInfo, int nBins, bool isHof, int size, int nxy_cell, int nt_cell)
{
	descInfo->nBins = nBins;
	descInfo->isHof = isHof;
	descInfo->nxCells = nxy_cell;
	descInfo->nyCells = nxy_cell;
	descInfo->ntCells = nt_cell;
	descInfo->dim = nBins*nxy_cell*nxy_cell;
	descInfo->height = size;
	descInfo->width = size;
}

void InitSeqInfo(SeqInfo* seqInfo, char* video)
{
	VideoCapture capture;
	capture.open(video);

	if(!capture.isOpened())
		fprintf(stderr, "Could not initialize capturing..\n");

	// get the number of frames in the video
	int frame_num = 0;
	while(true) {
		Mat frame;
		capture >> frame;

		if(frame.empty())
			break;

		if(frame_num == 0) {
			seqInfo->width = frame.cols;
			seqInfo->height = frame.rows;
		}

		frame_num++;
    }
	seqInfo->length = frame_num;
}

void usage()
{
	fprintf(stderr, "Extract dense trajectories from a video\n\n");
	fprintf(stderr, "Usage: DenseTrack video_file [options]\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, "  -h                        Display this message and exit\n");
	fprintf(stderr, "  -S [start frame]          The start frame to compute feature (default: S=0 frame)\n");
	fprintf(stderr, "  -E [end frame]            The end frame for feature computing (default: E=last frame)\n");
	fprintf(stderr, "  -L [trajectory length]    The length of the trajectory (default: L=15 frames)\n");
	fprintf(stderr, "  -W [sampling stride]      The stride for dense sampling feature points (default: W=5 pixels)\n");
	fprintf(stderr, "  -N [neighborhood size]    The neighborhood size for computing the descriptor (default: N=32 pixels)\n");
	fprintf(stderr, "  -s [spatial cells]        The number of cells in the nxy axis (default: nxy=2 cells)\n");
	fprintf(stderr, "  -t [temporal cells]       The number of cells in the nt axis (default: nt=3 cells)\n");
	fprintf(stderr, "  -A [scale number]         The number of maximal spatial scales (default: 8 scales)\n");
	fprintf(stderr, "  -I [initial gap]          The gap for re-sampling feature points (default: 1 frame)\n");
}

bool arg_parse(int argc, char** argv)
{
	int c;
	bool flag = false;
	char* executable = basename(argv[0]);
	while((c = getopt (argc, argv, "hS:E:L:W:N:s:t:A:I:")) != -1)
	switch(c) {
		case 'S':
		start_frame = atoi(optarg);
		flag = true;
		break;
		case 'E':
		end_frame = atoi(optarg);
		flag = true;
		break;
		case 'L':
		track_length = atoi(optarg);
		break;
		case 'W':
		min_distance = atoi(optarg);
		break;
		case 'N':
		patch_size = atoi(optarg);
		break;
		case 's':
		nxy_cell = atoi(optarg);
		break;
		case 't':
		nt_cell = atoi(optarg);
		break;
		case 'A':
		scale_num = atoi(optarg);
		break;
		case 'I':
		init_gap = atoi(optarg);
		break;	

		case 'h':
		usage();
		exit(0);
		break;

		default:
		fprintf(stderr, "error parsing arguments at -%c\n  Try '%s -h' for help.", c, executable );
		abort();
	}
	return flag;
}

#endif /*INITIALIZE_H_*/
