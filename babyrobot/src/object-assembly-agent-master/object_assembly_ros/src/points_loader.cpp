

#include <stdlib.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector> 
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <dbot_ros_msgs/ObjectsState.h>
#include "points02.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test01_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<dbot_ros_msgs::ObjectsState>("/particle_tracker/objects_state", 0);

    double quaternion_vals[] = 
        {0,0,0,1,
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0.7071,0.7071,0,0, //4
        0.7071,0,0.7071,0,
        0.7071,0,0,0.7071,
        0,0.7071,0.7071,0,
        0,0.7071,0,0.7071, //8
        0,0,0.7071,0.7071,
        -0.7071,0.7071,0,0,
        -0.7071,0,0.7071,0,
        -0.7071,0,0,0.7071, //12
        0,-0.7071,0.7071,0,
        0,-0.7071,0,0.7071,
        0,0,-0.7071,0.7071,
        0.5,0.5,0.5,0.5, //16
        -0.5,0.5,0.5,0.5,
        0.5,-0.5,0.5,0.5,
        0.5,0.5,-0.5,0.5,
        -0.5,-0.5,-0.5,0.5, //20
        -0.5,-0.5,0.5,0.5,
        -0.5,0.5,-0.5,0.5,
        0.5,-0.5,-0.5,0.5};
    int rand_ind[12];
    std::srand(time(NULL));
//std::vector<int> v;
    for (int i=0;i<12;i++){
        rand_ind[i] = rand() % 24 ;
        std::cout << i << ": " << rand_ind[i] << "\n";
        //v.push_back(rand_ind[i]);
    }

    int ind=0;
    for (int t=0;t<1200;t++)
    {
        dbot_ros_msgs::ObjectsState objects_state_message;
        for (int i = 0; i < 12; i++)
        {
            dbot_ros_msgs::ObjectState temp_object_state_message;
            geometry_msgs::PoseStamped pose_stamped;
            //geometry_msgs::Pose pose;
            pose_stamped.pose.position.x = points[ind++];
            pose_stamped.pose.position.y = points[ind++];
            pose_stamped.pose.position.z = points[ind++];
            if (0)//(i==1||i==2||i==7||i==11)
            {
                pose_stamped.pose.orientation.x = atof(argv[2]);
                pose_stamped.pose.orientation.y = atof(argv[3]);
                pose_stamped.pose.orientation.z = atof(argv[4]);
                pose_stamped.pose.orientation.w = atof(argv[5]);
            }
            else
            {
                pose_stamped.pose.orientation.x = quaternion_vals[4*rand_ind[i]];
                pose_stamped.pose.orientation.y = quaternion_vals[4*rand_ind[i]+1];
                pose_stamped.pose.orientation.z = quaternion_vals[4*rand_ind[i]+2];
                pose_stamped.pose.orientation.w = quaternion_vals[4*rand_ind[i]+3];
            }
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "no_frame";

            temp_object_state_message.pose = pose_stamped;

            //temp_object_state_message.pose = pose[i];
            temp_object_state_message.ori.name = "no_name.obj";
            temp_object_state_message.ori.directory = "no_directory";
            temp_object_state_message.ori.package = "no_package";

            temp_object_state_message.name = "no_name";

            objects_state_message.objects_state.push_back (temp_object_state_message);
        }
        if (t%10==0) std::cout << "Time: " << t << "\n";
        objects_state_message.active_object_id = 0;
        pub.publish(objects_state_message);
        usleep(atoi(argv[1]));
        ros::spinOnce();
    }

    //ros::spin();
    return 0;
}

