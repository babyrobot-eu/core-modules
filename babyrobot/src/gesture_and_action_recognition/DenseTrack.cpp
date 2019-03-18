#include "DenseTrack.h"
#include "Initialize.h"
#include "Descriptors.h"
#include "OpticalFlow.h"
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <typeinfo>
#include <ctime>
#include <signal.h>
#include "Recognizer.h"

#include <time.h>

using namespace cv;

int show_track = 1; // set show_track = 1, if you want to visualize the trajectories
int person_track = 1;
int lock_person = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "action_dense_track");
    ros::NodeHandle node_handle;
    signal(SIGINT, mySigintHandler);
//    const char * codebook_file = "/home/nick/catkin_ws/src/action_dense_track/src/data/validation_scenario_w_bm_cleaner_ComeHere/codebook_MBH";
//    const char * encoded_features_file = "/home/nick/catkin_ws/src/action_dense_track/src/data/validation_scenario_w_bm_cleaner_ComeHere/encoded_features_MBH";
//    const char * norm_file = "/home/nick/catkin_ws/src/action_dense_track/src/data/validation_scenario_w_bm_cleaner_ComeHere/norm_MBH";
//    std::string model_files = "/home/nick/catkin_ws/src/action_dense_track/src/data/validation_scenario_w_bm_cleaner_ComeHere/model";
////    std::string vocabulary[] = {"Help","WantStandUp","PerformTask","WantSitDown","ComeCloser","ComeHere", "GoStraight", "Park", "Stop", "TurnLeft","TurnRight","WhereAmI"};
//    std::string vocabulary[] = {"Help","WantStandUp","ComeCloser","Park","WhatTime", "GoAway", "BM"};
//    std::vector<std::string> vocabulary_vec(vocabulary, vocabulary + sizeof(vocabulary) / sizeof(std::string)); 
//    const int num_classes = 7;
    int decim_factor = 8;
	int per_x=0,per_y=0,per_w=0,per_h=0;
    int i, c;
    unsigned int t0, t1;
    
    // Reading from Parameter Server
    int num_classes, num_classes_p, action_type;
    std::string codebook_file, encoded_features_file, norm_file, model_files, kinect_topic, activity_det_topic, person_det_topic, fusion_link_topic, logfile_dir, codebook_file_p, encoded_features_file_p, norm_file_p, model_files_p;
    node_handle.getParam("/num_classes", num_classes);
    node_handle.getParam("/codebook_file", codebook_file);
    node_handle.getParam("/encoded_features_file", encoded_features_file);
    node_handle.getParam("/norm_file", norm_file);
    node_handle.getParam("/model_files", model_files);
    std::vector<std::string> vocabulary(num_classes);
    node_handle.getParam("/class_names", vocabulary);
    
    node_handle.getParam("/num_classes_p", num_classes_p);
    node_handle.getParam("/codebook_file_p", codebook_file_p);
    node_handle.getParam("/encoded_features_file_p", encoded_features_file_p);
    node_handle.getParam("/norm_file_p", norm_file_p);
    node_handle.getParam("/model_files_p", model_files_p);
    std::vector<std::string> vocabulary_p(num_classes_p);
    node_handle.getParam("/class_names_p", vocabulary_p);
    
    node_handle.getParam("/kinect_topic", kinect_topic);
    node_handle.getParam("/activity_det_topic", activity_det_topic);
	node_handle.getParam("/person_det_topic", person_det_topic);
    node_handle.getParam("/fusion_link_topic", fusion_link_topic);
    node_handle.getParam("/logfile_dir", logfile_dir);
    

    std::string logfilename = logfile_dir + "/logfile_" + currentDateTime();
    std::cout << "Logfile: " << logfilename << std::endl;
    std::fstream logfile_fs;
    logfile_fs.open(logfilename.c_str(), std::fstream::out);

    
//    suc = node_handle.getParam("/classes_names", v);
//    std::vector<std::string> v(7);
//    bool suc = node_handle.getParam("/classes/names", v);
//    std::cout << suc << " " << v[0] << " " << v[1] << std::endl;
    //
    frame_manager manager(node_handle, &logfile_fs, kinect_topic, activity_det_topic, person_det_topic);
    recognizer r(codebook_file.c_str(), encoded_features_file.c_str(), norm_file.c_str(), model_files.c_str(), num_classes);
    recognizer r_p(codebook_file_p.c_str(), encoded_features_file_p.c_str(), norm_file_p.c_str(), model_files_p.c_str(), num_classes_p);
    fusion_link _fusion_link(node_handle, vocabulary, num_classes, fusion_link_topic);
    fusion_link _fusion_link_p(node_handle, vocabulary_p, num_classes_p, fusion_link_topic);
    
        std::cout << "There are " << num_classes << " available classes:" << std::endl;
        for (i=0; i<num_classes; i++){
            std::cout  << vocabulary[i] << " ";
        }
        std::cout << std::endl;
        
        std::cout << "There are " << num_classes_p << " available classes:" << std::endl;
        for (i=0; i<num_classes_p; i++){
            std::cout  << vocabulary_p[i] << " ";
        }
        std::cout << std::endl;
    
    //ros::spin();
    //return(0);
    
    //VideoCapture capture;
    //char* video = argv[1];
    //string videoFile = string(video);
    //string videoPath = videoFile.substr(0,videoFile.find_last_of("/\\"));


    int flag = arg_parse(argc, argv);
    //capture.open(video);

    //if(!capture.isOpened()) {
    //	fprintf(stderr, "Could not initialize capturing..\n");
    //	return -1;
    //}

    int frame_num = 0;
    TrackInfo trackInfo;
    DescInfo hogInfo, hofInfo, mbhInfo;

    InitTrackInfo(&trackInfo, track_length, init_gap);
    InitDescInfo(&hogInfo, 8, false, patch_size, nxy_cell, nt_cell);
    InitDescInfo(&hofInfo, 9, true, patch_size, nxy_cell, nt_cell);
    InitDescInfo(&mbhInfo, 8, false, patch_size, nxy_cell, nt_cell);

//    SeqInfo seqInfo;
//    //InitSeqInfo(&seqInfo, video);
//    seqInfo.width = 640;
//    seqInfo.height = 480;
//    seqInfo.length = 100;
//    start_frame = 0;
//    end_frame = 100;
//    if(flag)
//            seqInfo.length = end_frame - start_frame + 1;

//	fprintf(stderr, "video size, length: %d, width: %d, height: %d\n", seqInfo.length, seqInfo.width, seqInfo.height);

    int frameCounter = 1;
//    char imageName [40];

    if(show_track == 1) {
        namedWindow("DenseTrack", 0);
//        namedWindow("Processing", 0);
    }

    Mat image, prev_grey, grey;

    std::vector<float> fscales(0);
    std::vector<Size> sizes(0);

    std::vector<Mat> prev_grey_pyr(0), grey_pyr(0), flow_pyr(0);
    std::vector<Mat> prev_poly_pyr(0), poly_pyr(0); // for optical flow

    std::vector<std::list<Track> > xyScaleTracks;
    int init_counter = 0; // indicate when to detect new feature points
    Mat frame, tmp;
    bool first_frame = true, ready = false;
//    FILE * fid = fopen("/home/nick/Desktop/out.txt", "w");
    std::ostringstream outStream;
    int num_valid_tracks;
    
    while(ros::ok()) {

	// get a new frame
	//capture >> frame;
	//if(frame.empty())
			//break;
        ros::spinOnce();
        if (manager.command==IGNORE){
			lock_person = 0;
            continue;
		}
        else if (manager.command==REJECT) {
            outStream.clear();
            outStream.str("");
            manager.acknowledge(manager.command);
            first_frame = true;
			lock_person = 0;
            continue;
        }
        else if ((manager.command==CLASSIFY)) {
            if (manager.frame_buffer.empty()) {
//                std::cout << "Classifying..." << std::endl;
				node_handle.getParam("/action_type", action_type);
				if (action_type==0) {
                r.recognize(&outStream);
//                std::cout << std::endl << "** Gesture Recognition: " << vocabulary[r.result] << " **" << " " << r.score << " " << num_valid_tracks << std::endl << std::endl;
                std::cout << std::endl << "** Gesture Recognition: " << vocabulary[r.result] << " **" << std::endl << std::endl;
                outStream.clear();
                outStream.str("");
                first_frame = true;
                t1 = clock();
				std::cout << "It took me " << double(t1-t0) / CLOCKS_PER_SEC << " secs to classify " << manager.process_frames_cnt \
			    << " frames (" << (manager.process_frames_cnt / (double(t1-t0) / CLOCKS_PER_SEC)) << ") fps" << std::endl;
                manager.acknowledge(manager.command);
                logfile_fs << vocabulary[r.result]  << " (" << double(t1-t0) / CLOCKS_PER_SEC << ") " << std::endl;
                //if (!(vocabulary[r.result]== "BM"))
                    _fusion_link.broadcast_result(r.probs, manager.start_time, manager.end_time);
                continue;}
                if (action_type==1) {
                r_p.recognize(&outStream);
//                std::cout << std::endl << "** Gesture Recognition: " << vocabulary[r.result] << " **" << " " << r.score << " " << num_valid_tracks << std::endl << std::endl;
                std::cout << std::endl << "** Action Recognition: " << vocabulary_p[r_p.result] << " **" << std::endl << std::endl;
                outStream.clear();
                outStream.str("");
                first_frame = true;
                t1 = clock();
				std::cout << "It took me " << double(t1-t0) / CLOCKS_PER_SEC << " secs to classify " << manager.process_frames_cnt \
			    << " frames (" << (manager.process_frames_cnt / (double(t1-t0) / CLOCKS_PER_SEC)) << ") fps" << std::endl;
                manager.acknowledge(manager.command);
                logfile_fs << vocabulary_p[r_p.result]  << " (" << double(t1-t0) / CLOCKS_PER_SEC << ") " << std::endl;
                //if (!(vocabulary_p[r_p.result]== "BM"))
                    _fusion_link_p.broadcast_result(r_p.probs, manager.start_time, manager.end_time);
                continue;}
            }
        }
        else if (manager.frame_buffer.empty()) {
            //std::cout << "The frame buffer is empty." << std::endl;
            continue;
        };
        
		//std::cout << manager.x << manager.y << (manager.h-manager.x) << (manager.w-manager.y) << std::endl;
        manager.frame_buffer.back().copyTo(frame);
        manager.frame_buffer.pop_back();
        manager.timestamp_buffer.pop_back();
		Mat croppedImage;
		if (person_track == 1){
			if (lock_person == 0){
				per_x = manager.x;
				per_y = manager.y;
				per_w = (manager.w-manager.x);
				per_h = (manager.h-manager.y);
				lock_person = 1;
			}
			Rect myROI(per_x, per_y, per_w, per_h);
			croppedImage = frame(myROI).clone();
		}
//        ready = false;
//        if (manager.frame_buffer.empty())
//            ready = true;
        
        if (decim_factor>1) {
            tmp = frame;
			if (person_track == 1){
				tmp = croppedImage;
			}
            pyrDown(tmp, frame, Size(tmp.cols/2, tmp.rows/2));
            if (decim_factor>2){
                tmp = frame;
                pyrDown(tmp, frame, Size(tmp.cols/2, tmp.rows/2));
			}
            if (decim_factor>4){
                tmp = frame;
                pyrDown(tmp, frame, Size(tmp.cols/2, tmp.rows/2));
			}
        }
            
//        if(show_track == 1) {
//            imshow( "Processing", frame);
//            c = cvWaitKey(25);
//            //imshow( "Incoming Image", r.frame_buffer.front());
//        }

        //continue;
//        std::cout << r.frame_buffer.size() << " " << frame.cols << " " << frame.rows << std::endl;

        /*if(frame_num < first_frame_frame || frame_num > end_frame) {
                frame_num++;
                continue;
        }*/

        if(first_frame) {
            t0=clock();
            first_frame = false;
            num_valid_tracks = 0;
            image.create(frame.size(), CV_8UC3);
            grey.create(frame.size(), CV_8UC1);
            prev_grey.create(frame.size(), CV_8UC1);

            InitPry(frame, fscales, sizes);

            BuildPry(sizes, CV_8UC1, prev_grey_pyr);
            BuildPry(sizes, CV_8UC1, grey_pyr);

            BuildPry(sizes, CV_32FC2, flow_pyr);
            BuildPry(sizes, CV_32FC(5), prev_poly_pyr);
            BuildPry(sizes, CV_32FC(5), poly_pyr);

            xyScaleTracks.resize(scale_num);

            frame.copyTo(image);
            cvtColor(image, prev_grey, CV_BGR2GRAY);

            for(int iScale = 0; iScale < scale_num; iScale++) {
                    if(iScale == 0)
                            prev_grey.copyTo(prev_grey_pyr[0]);
                    else
                            resize(prev_grey_pyr[iScale-1], prev_grey_pyr[iScale], prev_grey_pyr[iScale].size(), 0, 0, INTER_LINEAR);

                    // dense sampling feature points
                    std::vector<Point2f> points(0);
                    DenseSample(prev_grey_pyr[iScale], points, quality, min_distance);

                    // save the feature points
                    std::list<Track>& tracks = xyScaleTracks[iScale];
                    for(i = 0; i < points.size(); i++)
                            tracks.push_back(Track(points[i], trackInfo, hogInfo, hofInfo, mbhInfo));
            }

            // compute polynomial expansion
            my::FarnebackPolyExpPyr(prev_grey, prev_poly_pyr, fscales, 7, 1.5);

            frame_num++;
            continue;
        }

        init_counter++;
        frame.copyTo(image);
        cvtColor(image, grey, CV_BGR2GRAY);

        // compute optical flow for all scales once
        my::FarnebackPolyExpPyr(grey, poly_pyr, fscales, 7, 1.5);
        my::calcOpticalFlowFarneback(prev_poly_pyr, poly_pyr, flow_pyr, 10, 2);
        
        
        for(int iScale = 0; iScale < scale_num; iScale++) {
                if(iScale == 0)
                        grey.copyTo(grey_pyr[0]);
                else
                        resize(grey_pyr[iScale-1], grey_pyr[iScale], grey_pyr[iScale].size(), 0, 0, INTER_LINEAR);

                int width = grey_pyr[iScale].cols;
                int height = grey_pyr[iScale].rows;

                // compute the integral histograms
                //DescMat* hogMat = InitDescMat(height+1, width+1, hogInfo.nBins);
                //HogComp(prev_grey_pyr[iScale], hogMat->desc, hogInfo);

                //DescMat* hofMat = InitDescMat(height+1, width+1, hofInfo.nBins);
                //HofComp(flow_pyr[iScale], hofMat->desc, hofInfo);

                DescMat* mbhMatX = InitDescMat(height+1, width+1, mbhInfo.nBins);
                DescMat* mbhMatY = InitDescMat(height+1, width+1, mbhInfo.nBins);
                MbhComp(flow_pyr[iScale], mbhMatX->desc, mbhMatY->desc, mbhInfo);

                // track feature points in each scale separately
                std::list<Track>& tracks = xyScaleTracks[iScale];
                for (std::list<Track>::iterator iTrack = tracks.begin(); iTrack != tracks.end();) {
                        int index = iTrack->index;
                        Point2f prev_point = iTrack->point[index];
                        int x = std::min<int>(std::max<int>(cvRound(prev_point.x), 0), width-1);
                        int y = std::min<int>(std::max<int>(cvRound(prev_point.y), 0), height-1);

                        Point2f point;
                        point.x = prev_point.x + flow_pyr[iScale].ptr<float>(y)[2*x];
                        point.y = prev_point.y + flow_pyr[iScale].ptr<float>(y)[2*x+1];

                        if(point.x <= 0 || point.x >= width || point.y <= 0 || point.y >= height) {
                                iTrack = tracks.erase(iTrack);
                                continue;
                        }

                        // get the descriptors for the feature point
                        RectInfo rect;
                        GetRect(prev_point, rect, width, height, hogInfo);
//                        GetDesc(hogMat, rect, hogInfo, iTrack->hog, index);
//                        GetDesc(hofMat, rect, hofInfo, iTrack->hof, index);
                        GetDesc(mbhMatX, rect, mbhInfo, iTrack->mbhX, index);
                        GetDesc(mbhMatY, rect, mbhInfo, iTrack->mbhY, index);
                        iTrack->addPoint(point);

                        // draw the trajectories at the first scale
                        if(show_track == 1 && iScale == 0)
                                DrawTrack(iTrack->point, iTrack->index, fscales[iScale], image);

                        // if the trajectory achieves the maximal length
                        if(iTrack->index >= trackInfo.length) {
                                std::vector<Point2f> trajectory(trackInfo.length+1);
                                for(int i = 0; i <= trackInfo.length; ++i)
                                        trajectory[i] = iTrack->point[i]*fscales[iScale];

                                float mean_x(0), mean_y(0), var_x(0), var_y(0), length(0);
                                if(IsValid(trajectory, mean_x, mean_y, var_x, var_y, length)) {
//                                        printf("%d\t%f\t%f\t%f\t%f\t%f\t%f\t", frame_num, mean_x, mean_y, var_x, var_y, length, fscales[iScale]);

                                        // for spatio-temporal pyramid
//                                        printf("%f\t", std::min<float>(std::max<float>(mean_x/float(seqInfo.width), 0), 0.999));
//                                        printf("%f\t", std::min<float>(std::max<float>(mean_y/float(seqInfo.height), 0), 0.999));
//                                        printf("%f\t", std::min<float>(std::max<float>((frame_num - trackInfo.length/2.0 - start_frame)/float(seqInfo.length), 0), 0.999));
//
//                                        // output the trajectory
//                                        for (int i = 0; i < trackInfo.length; ++i)
//                                                printf("%f\t%f\t", trajectory[i].x,trajectory[i].y);
//
//                                        PrintDesc(iTrack->hog, hogInfo, trackInfo);
//                                        PrintDesc(iTrack->hof, hofInfo, trackInfo);
//                                        PrintDesc(iTrack->mbhX, mbhInfo, trackInfo);
//                                        PrintDesc(iTrack->mbhY, mbhInfo, trackInfo);
//                                        printf("\n");
                                        num_valid_tracks++;
                                	PrintDesc2Stream(outStream, iTrack->mbhX, mbhInfo, trackInfo);
					PrintDesc2Stream(outStream, iTrack->mbhY, mbhInfo, trackInfo);
					outStream << std::endl;
//                                        PrintDesc2File(fid, iTrack->mbhX, mbhInfo, trackInfo);
//                                        PrintDesc2File(fid, iTrack->mbhY, mbhInfo, trackInfo);
//                                        fprintf(fid, "\n");
                                }

                                iTrack = tracks.erase(iTrack);
                                continue;
                        }
                        ++iTrack;
                }
                //ReleDescMat(hogMat);
                //ReleDescMat(hofMat);
                ReleDescMat(mbhMatX);
                ReleDescMat(mbhMatY);

                if(init_counter != trackInfo.gap)
                        continue;

                // detect new feature points every initGap frames
                std::vector<Point2f> points(0);
                for(std::list<Track>::iterator iTrack = tracks.begin(); iTrack != tracks.end(); iTrack++)
                        points.push_back(iTrack->point[iTrack->index]);

                DenseSample(grey_pyr[iScale], points, quality, min_distance);
                // save the new feature points
                for(i = 0; i < points.size(); i++)
                        tracks.push_back(Track(points[i], trackInfo, hogInfo, hofInfo, mbhInfo));
        }

        init_counter = 0;
        grey.copyTo(prev_grey);
        for(i = 0; i < scale_num; i++) {
                grey_pyr[i].copyTo(prev_grey_pyr[i]);
                poly_pyr[i].copyTo(prev_poly_pyr[i]);
        }

        frame_num++;

        if( show_track == 1 ) {

                //sprintf( imageName,"figures/img%04d.jpg", frameCounter);
                //string figFileName = videoPath + imageName;
//                frameCounter ++;
                //imwrite( figFileName, image);
                //imwrite( imageName, image);
                //sprintf( imageName,"figures/img_%04d.jpg", frameCounter);
                //imwrite( imageName, frame);
                imshow( "DenseTrack", image);
                c = cvWaitKey(25);
                if((char)c == 27) break;
        }
    }

    if( show_track == 1 )
	destroyWindow("DenseTrack");

	return 0;
}
