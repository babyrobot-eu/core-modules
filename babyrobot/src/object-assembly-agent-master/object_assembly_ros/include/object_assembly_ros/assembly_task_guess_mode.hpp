#include <assembly_subtask.hpp>
#include <cstdlib>

class AssemblyTask
{

public:
    AssemblyTask(
        int id,
        int num_parts,
        std::vector<std::vector<int>> connnection_pairs,
        int num_connections,
        std::vector<AssemblySubtask> subtasks,
        int max_particles,
        int all_particles,
        int connection_list_offset,
        std::string connection_vector_topic);
    
    //returns current subtask
    double evaluate_task(std::vector<tf::Transform> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list);

    void set_connections(std::vector<object_assembly_msgs::ConnectionInfo> &connection_list);

    tf::Quaternion calculate_prerotation();

    void publish_connection_vector();

    int num_connections_;

    //struct AssemblySubgraph
    //{
    //    std::vector<tf::Quaternion> rotations;
    //    std::vector<int> nodes;
    //};

private:
    ros::NodeHandle node_handle_;
    ros::Publisher connection_vector_publisher_;
    //int current_subtask_ = 0;
    const int task_id_;
    const int num_parts_;
    const int connection_list_offset_;
    int subgraph_max_index_ = -1;
    std::vector<AssemblySubtask> subtasks_;
    int max_particles_;
    int all_particles_;
    std::vector<std::vector<int>> connection_pairs_;
    std::vector<bool> connection_status_vector_;
    //std::vector<AssemblySubgraph> subgraph_list_;
    std::vector<int> part_subgraph_index_; //which subgraph each part belongs to (-1 means none)
    std::vector<double> scores_;

    std::list<int> zero_score_queue_;
    std::vector<int> zero_score_spotlight_times_;

    std::vector<tf::Quaternion> rotations_;
    //std::vector<int> rotation_index;

};
