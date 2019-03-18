//#include <Eigen/Dense>

#include <fstream>
#include <ctime>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include <assembly_task_guidance_mode.hpp>

#include <dbot_ros_msgs/ObjectsState.h>
#include <object_assembly_msgs/FetchSurfaceParams.h>
#include <object_assembly_msgs/ConnectionInfoList.h>

class AssemblyAgentNode
{

public:

    AssemblyAgentNode(AssemblyTask task, std::string connection_info_list_topic, std::vector<object_assembly_msgs::ConnectionInfo> connection_list)
        : task_(task), connection_list_(connection_list), node_handle_("~")
    {
        std::cout << "Creating Assembly Agent Node\n";

        connection_list_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionInfoList>(connection_info_list_topic, 0);
        connection_vector_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionVector>("connection_vector", 0);
    }

    void publish_connection_vector(int current_subtask)
    {
        object_assembly_msgs::ConnectionVector connection_vector_msg;
        for (int i = 0; i < connection_list_.size(); i++)
            connection_vector_msg.v.push_back((char)(i < current_subtask));
        connection_vector_publisher_.publish(connection_vector_msg);
    }

    void assembly_agent_callback(const dbot_ros_msgs::ObjectsState& state)
    {
	    std::vector<geometry_msgs::Pose> poses;
	    for (int i = 0; i < state.objects_state.size(); i++) {        
	        poses.push_back (state.objects_state[i].pose.pose);
        }
	    current_object_poses_ = poses;
	    int current_subtask = task_.evaluate_task(current_object_poses_, connection_list_);
        if (current_subtask == task_.num_subtasks_)
        {
            std::cout << "You have completed the assembly task!!" << '\n';
        }
        else
        {
            std::cout << task_.subtask_description(current_subtask) << " (subtask: " << current_subtask+1 << ")\n";
        }
        object_assembly_msgs::ConnectionInfoList connections_msg;
        connections_msg.connections = connection_list_;
        connection_list_publisher_.publish(connections_msg);
        publish_connection_vector(current_subtask);
    }

private:
    AssemblyTask task_;
    ros::NodeHandle node_handle_;
    std::vector<std::string> object_names_;
    std::vector<geometry_msgs::Pose> current_object_poses_;
    geometry_msgs::Pose current_relative_pose_;
    std::vector<object_assembly_msgs::ConnectionInfo> connection_list_;
    ros::Publisher connection_list_publisher_;
    ros::Publisher connection_vector_publisher_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "assembly_agent");
    ros::NodeHandle nh("~");

    tf::Vector3 up_vector;

    std::string surface_params_service;
    nh.getParam("surface_params_service", surface_params_service);
    ros::ServiceClient client = nh.serviceClient<object_assembly_msgs::FetchSurfaceParams>(surface_params_service);
    object_assembly_msgs::FetchSurfaceParams srv;
    srv.request.recalculate = true;
    while (!client.call(srv))
    {
        ROS_ERROR("Failed to call background removal service. Trying again...");
        usleep(1000000);
    }
    up_vector.setX(srv.response.surface_parameters.a1);
    up_vector.setY(-1.0);
    up_vector.setZ(srv.response.surface_parameters.a3);
    up_vector.normalize();

    // object data
    std::vector<std::string> object_names;
    std::vector<std::string> object_symmetries;
    std::string object_pose_topic;
    std::vector<std::string> descriptions;

    //task data
    int num_subtasks, num_checks;
    std::vector<AssemblySubtask> subtasks;

    /* ------------------------------ */
    /* -     Read out data          - */
    /* ------------------------------ */
    // get object names
    nh.getParam("objects", object_names);
    nh.getParam("object_symmetries", object_symmetries);
    nh.getParam("object_pose_topic", object_pose_topic);
    nh.getParam("number_of_checks", num_checks);

    std::vector<object_assembly_msgs::ConnectionInfo> connection_list;
    // get task data
    nh.getParam("number_of_subtasks", num_subtasks);
    int N;
    nh.getParam("N", N);
    double ymax;
    nh.getParam("ymax", ymax);
    int max_particles;
    nh.getParam("max_particles", max_particles);
    int all_particles;
    nh.getParam("all_particles", all_particles);

    std::vector<std::vector<int>> connection_pairs;

    for (int i=1; i<=num_subtasks; i++) {
        std::string connection_type;
	    int first_object;
    	int second_object;

	// subtask shorthand prefix
        std::string subtask_pre = std::string("subtasks/") + "subtask" + std::to_string(i) + "/";

        std::string description;
        nh.getParam(subtask_pre + "description", description);
        descriptions.push_back(description);

	    nh.getParam(subtask_pre + "object", second_object);
	    nh.getParam(subtask_pre + "relative_object", first_object);
	    nh.getParam(subtask_pre + "connection_type", connection_type);
        --first_object;
        --second_object;
        std::vector<int> temp;
        temp.push_back(first_object);
        temp.push_back(second_object);
        connection_pairs.push_back(temp);

	    std::vector<double> relative_pose;
	    nh.getParam(subtask_pre + "relative_pose", relative_pose);
        tf::Vector3 relative_position;
        relative_position.setValue(relative_pose[0],
                                   relative_pose[1],
                                   relative_pose[2]);
        tf::Quaternion relative_orientation = 
                tf::Quaternion(relative_pose[3],
                               relative_pose[4],
                               relative_pose[5],
                               relative_pose[6]);
        std::vector<double> margin;
	    nh.getParam(subtask_pre + "margin", margin);
        std::string frame;
        nh.getParam(subtask_pre + "frame", frame);
	
        AssemblySubtask subtask(i-1,
                                num_checks,
                                first_object,
    				            second_object,
                                frame,
    				            relative_position,
                                relative_orientation,
                                margin,
    				            connection_type,
                                object_symmetries,
                                max_particles,
                                up_vector,
                                N,
                                ymax);
	    subtasks.push_back(subtask);

        //build connection messages to save time later
        //TODO move to AssemblyAgentNode, because the relative pose
        //in the message should change when objects are symmetrical
        object_assembly_msgs::ConnectionInfo connection;
        connection.part = second_object;
        connection.relative_part = first_object;
        connection.relative_pose.position.x = relative_pose[0];
        connection.relative_pose.position.y = relative_pose[1];
        connection.relative_pose.position.z = relative_pose[2];
        connection.relative_pose.orientation.x = relative_pose[3];
        connection.relative_pose.orientation.y = relative_pose[4];
        connection.relative_pose.orientation.z = relative_pose[5];
        connection.relative_pose.orientation.w = relative_pose[6];
        connection.num_particles = 0;
        connection.first_particle = (i-1)*max_particles;
        connection_list.push_back(connection);
    }

    bool cubic = (object_symmetries[0]=="cubic");

    AssemblyTask task(object_names.size(), connection_pairs, num_subtasks, subtasks, max_particles, all_particles, descriptions, cubic);

    std::string connection_info_list_topic;
    nh.getParam("connection_info_list_topic",
                connection_info_list_topic);

    AssemblyAgentNode assembly_agent_node(task,
                connection_info_list_topic, connection_list);

    ros::Subscriber subscriber = nh.subscribe(
        object_pose_topic, 1, &AssemblyAgentNode::assembly_agent_callback, &assembly_agent_node);

    ros::spin();

    return 0;
}
