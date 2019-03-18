#include <fstream>
#include <ctime>
#include <memory>
#include <thread>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/package.h>

#include <assembly_task_learning_mode.hpp>

#include <dbot_ros_msgs/ObjectsState.h>
#include <object_assembly_msgs/FetchSurfaceParams.h>
#include <object_assembly_msgs/ConnectionInfoList.h>

#include <learning_mode_ui.hpp>


class AssemblyAgentNode
{

public:

    AssemblyAgentNode(std::vector<std::string> object_names, std::vector<AssemblyTask> tasks, std::string connection_info_list_topic, std::vector<std::vector<object_assembly_msgs::ConnectionInfo>> connection_lists, int argc, char** argv, bool flip_view, tf::Transform camera2user)
        : object_names_(object_names), tasks_(tasks), node_handle_("~"), num_tasks_(tasks.size()), argc_(argc), argv_(argv), flip_view_(flip_view), camera2user_(camera2user)
    {
        std::cout << "Creating Assembly Agent Node\n";
        std::cout << "Choose a sentence (1-" << num_tasks_ << ")\n";
        for (int i = 0; i < num_tasks_; i++)
        {
            std::cout << i+1 << ": ";
            tasks_[i].display_english_sentence();
        }
        do
        {        
            std::cin >> current_task_;
            if (current_task_ > num_tasks_ || current_task_ <= 0)
            {
                std::cout << "Invalid selection\n";
            }  
        } while (current_task_ > num_tasks_ || current_task_ <= 0);
        --current_task_; 
        connection_list_ = connection_lists[current_task_];

        connection_list_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionInfoList>(connection_info_list_topic, 0);

        std::vector<std::string> filenames;
        for(int i = 0; i< object_names_.size(); i++)
            filenames.push_back("/home/jack/code_test2/catkinws2/src/object_assembly_ros/resource/cube30mm24vert.obj");
        use_png_texture = true;
        ui = new LearningModeUI(filenames);
        for (int i = 0; i < ui->meshes_.size(); i++)
        {
            std::string temp("/home/jack/catkin_ws/src/object_assembly_ros/output/" + tasks_[current_task_].words_shuffled_[i] + ".png");
            ui->meshes_[i].img_name = new char [temp.length()+1];   
            std::strcpy(ui->meshes_[i].img_name, temp.c_str());
            //for (int j = 0; j < ui->meshes_[i].vertices.size(); j++)
            //    ui->meshes_[i].set_uniform_colour(glm::vec3(0.3,0.3,0.0)); //doesn't do anything
        }
        //uis.push_back(ui);
        //This should be ok; a mutex is locked when entering new poses
        std::thread (BaseUI::gui_main_static, argc_, argv_).detach();
        std::cout << "\nCreate the sentence:\n";
        tasks_[current_task_].display_english_sentence();
    }

    ~AssemblyAgentNode()
    {
        delete ui;
    }

    void display_success_msg()
    {
        std::cout << "CORRECT!\n";
        usleep(5000000);
    }

    void assembly_agent_callback(const dbot_ros_msgs::ObjectsState& state)
    {
	    std::vector<geometry_msgs::Pose> poses;
	    for (int i = 0; i < state.objects_state.size(); i++) {        
	        poses.push_back (state.objects_state[i].pose.pose);
        }

	    current_object_poses_ = poses;
        std::vector<tf::Transform> tf_poses;
        for (int i = 0; i < current_object_poses_.size(); i++)
        {
            tf::Transform single_pose(tf::Quaternion (
                                        current_object_poses_[i].orientation.x,
                                        current_object_poses_[i].orientation.y,
                                        current_object_poses_[i].orientation.z,
                                        current_object_poses_[i].orientation.w),
                                      tf::Vector3 (
                                        current_object_poses_[i].position.x,
                                        current_object_poses_[i].position.y,
                                        current_object_poses_[i].position.z));
            if (flip_view_)
            {
                tf_poses.push_back(camera2user_ * single_pose);
            }
            else
            {
                tf_poses.push_back(single_pose);
            }
        }
        ui->update_poses(tf_poses);

        if (tasks_[current_task_].evaluate_task(tf_poses, connection_list_))
        {
            display_success_msg();
            exit(1);
        }

        object_assembly_msgs::ConnectionInfoList connections_msg;
        connections_msg.connections = connection_list_;
        connection_list_publisher_.publish(connections_msg);
    }

private:
    const int num_tasks_;
    int current_task_ = -1;
    bool flip_view_;
    tf::Transform camera2user_;
    std::vector<AssemblyTask> tasks_;
    //int max_particles_;
    ros::NodeHandle node_handle_;
    std::vector<std::string> object_names_;
    std::vector<geometry_msgs::Pose> current_object_poses_;
    geometry_msgs::Pose current_relative_pose_;
    std::vector<object_assembly_msgs::ConnectionInfo> connection_list_;
    ros::Publisher connection_list_publisher_;
    LearningModeUI *ui;
    //std::vector<LearningModeUI*> uis;
    int argc_;
    char** argv_;
    int num_msgs_received_ = 0;
};

int find_position(std::vector<int> vector_i, int to_find)
{
    for (int i = 0; i < vector_i.size(); i++)
    {
        if (vector_i[i] == to_find)
            return i;
    }
    return -1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "assembly_agent");
    ros::NodeHandle nh("~");
    std::srand ( unsigned ( std::time(0) ) );

    tf::Vector3 up_vector;

    std::string surface_params_service;
    nh.getParam("surface_params_service", surface_params_service);
    ros::ServiceClient client = nh.serviceClient<object_assembly_msgs::FetchSurfaceParams>(surface_params_service);
    object_assembly_msgs::FetchSurfaceParams srv;
    srv.request.recalculate = true;
    while (!client.call(srv))
    {
        ROS_ERROR("Failed to call background removal service. Trying again...");
        usleep(500000);
    }
    up_vector.setX(srv.response.surface_parameters.a1);
    up_vector.setY(-1.0);
    up_vector.setZ(srv.response.surface_parameters.a3);

/*
    //TODO: REMOVE NEXT 3 LINES WHEN FINISHED EXPERIMENTING
    up_vector.setX(0.0);
    up_vector.setY(-1.0);
    up_vector.setZ(0.0);
    up_vector.normalize();
*/

    std::vector<std::string> object_symmetries;
    std::vector<std::string> task_names;
    std::string object_pose_topic;
    std::vector<std::string> descriptions;
    std::string image_output_dir;

    int num_boxes, num_checks, num_sentences;
    std::vector<AssemblyTask> tasks;

    /* ------------------------------ */
    /* -     Read out data          - */
    /* ------------------------------ */
    nh.getParam("number_of_boxes", num_boxes);
    for (int i = 0; i < num_boxes; i++)
    {
        object_symmetries.push_back("cubic");
    }
    nh.getParam("object_pose_topic", object_pose_topic);
    nh.getParam("number_of_checks", num_checks);
    nh.getParam("number_of_sentences", num_sentences);
    nh.getParam("image_output_dir", image_output_dir);
    if (image_output_dir.back() != '/')
        image_output_dir.push_back('/');
    std::vector<double> margin;
    nh.getParam("margin", margin);
    int max_particles;
    nh.getParam("max_particles", max_particles);
    int all_particles;
    nh.getParam("all_particles", all_particles);
    double box_size;
    nh.getParam("box_size", box_size);
    bool shuffle;
    nh.getParam("shuffle", shuffle);
    bool flip_view;
    nh.getParam("flip_view", flip_view);
    tf::Transform camera2user;
    camera2user.setIdentity();
    if (flip_view)
    {
        std::vector<double> camera2user_values;
        nh.getParam("camera_to_user_transform", camera2user_values);
        camera2user = tf::Transform(tf::Quaternion (
                                        camera2user_values[3],
                                        camera2user_values[4],
                                        camera2user_values[5],
                                        camera2user_values[6]),
                                      tf::Vector3 (
                                        camera2user_values[0],
                                        camera2user_values[1],
                                        camera2user_values[2]));
    }
    int N;
    nh.getParam("N", N);
    double ymax;
    nh.getParam("ymax", ymax);

    std::vector<std::vector<object_assembly_msgs::ConnectionInfo>> connection_lists;

    for (int i = 1; i <= num_sentences; i++)
    {
        std::string sentence_pre = std::string("sentence") + std::to_string(i) + "/";
        std::string spanish, english;
        std::vector<std::string> valid_words, other_words;
        nh.getParam(sentence_pre + "Spanish", spanish);
        nh.getParam(sentence_pre + "English", english);
        nh.getParam(sentence_pre + "other_words", other_words);
        std::istringstream iss(spanish);
        while (iss)
        {
            std::string sub;
            iss >> sub;
            valid_words.push_back(sub);
        }
        valid_words.pop_back();
        if (valid_words.size() + other_words.size() != num_boxes)
        {
            ROS_ERROR("Wrong number of words in sentence %d", i);
            exit(-1);
        }

        std::vector<int> rand_perm;
        for (int i = 0; i < num_boxes; i++)
            rand_perm.push_back(i);
        if (shuffle)
            std::random_shuffle(rand_perm.begin(), rand_perm.end());
        std::vector<std::string> words(valid_words);
        for (int i = 0; i < other_words.size(); i++)
            words.push_back(other_words[i]);
        std::vector<std::string> words_shuffled;
        for (int i = 0; i < words.size(); i++)
            words_shuffled.push_back(words[rand_perm[i]]);

        std::vector<AssemblySubtask> subtasks;
        std::vector<std::vector<int>> connection_pairs;
        std::vector<object_assembly_msgs::ConnectionInfo> connection_list;
        for (int j = 1; j < valid_words.size(); j++)
        {
            std::vector<int> temp;
            temp.push_back(find_position(rand_perm, j-1));
            temp.push_back(find_position(rand_perm, j));
            connection_pairs.push_back(temp);

            tf::Vector3 relative_position(-box_size, 0.0, 0.0);
            tf::Quaternion relative_orientation(0.0, 0.0, 0.0, 1.0);
            AssemblySubtask subtask(j-1,
                                    num_checks,
                                    temp[0],
        				            temp[1],
                                    "no_frame",
        				            relative_position,
                                    relative_orientation,
                                    margin,
        				            "place",
                                    object_symmetries,
                                    //all_particles/(valid_words.size()-1), 
                                    max_particles,
                                    up_vector,
                                    N,
                                    ymax);
            subtasks.push_back(subtask);

            object_assembly_msgs::ConnectionInfo connection;
            connection.part = temp[1];
            connection.relative_part = temp[0];
            connection.relative_pose.position.x = box_size;
            connection.relative_pose.position.y = 0.0;
            connection.relative_pose.position.z = 0.0;
            connection.relative_pose.orientation.x = 0.0;
            connection.relative_pose.orientation.y = 0.0;
            connection.relative_pose.orientation.z = 0.0;
            connection.relative_pose.orientation.w = 1.0;
            connection.num_particles = 0;
            connection.first_particle = (j-1) * all_particles / (valid_words.size()-1); //0
            connection_list.push_back(connection);
        }

        AssemblyTask task(num_boxes, connection_pairs, valid_words.size() - 1, subtasks, max_particles, all_particles, words_shuffled, english, image_output_dir);
        tasks.push_back(task);
        connection_lists.push_back(connection_list);
    }


    std::string connection_info_list_topic;
    nh.getParam("connection_info_list_topic",
                connection_info_list_topic);
    std::vector<std::string> object_names(num_boxes, "box");
    AssemblyAgentNode assembly_agent_node(object_names, tasks,
                connection_info_list_topic, connection_lists, argc, argv, flip_view, camera2user);

    ros::Subscriber subscriber = nh.subscribe(
        object_pose_topic, 1, &AssemblyAgentNode::assembly_agent_callback, &assembly_agent_node);
    ros::spin();

    return 0;
}
