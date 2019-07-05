#include <assembly_subtask.hpp>

class AssemblyTask
{

public:
    AssemblyTask(
        int num_parts,
        std::vector<std::vector<int>> connection_pairs,
        int num_connections,
        std::vector<AssemblySubtask> subtasks,
        int max_particles,
        int all_particles,
        std::vector<std::string> words,
        std::string english,
        std::string image_output_dir);
    
    //returns current subtask
    bool evaluate_task(std::vector<tf::Transform> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list);

    tf::Quaternion calculate_prerotation();

    void create_word_images();

    void display_english_sentence();

    void publish_connection_vector();

    int num_connections_;
    std::vector<std::string> words_shuffled_;


    struct AssemblySubgraph
    {
        std::vector<tf::Quaternion> rotations;
        std::vector<int> nodes;
    };

private:
    ros::NodeHandle node_handle_;
    ros::Publisher connection_vector_publisher_;
    //int current_subtask_ = 0;
    const int num_parts_;
    const std::string image_output_dir_;
    std::string english_;
    int subgraph_max_index_ = -1;
    std::vector<AssemblySubtask> subtasks_;
    int max_particles_;
    int all_particles_;
    std::vector<std::vector<int>> connection_pairs_;
    std::vector<bool> connection_status_vector_;
    std::vector<AssemblySubgraph> subgraph_list_;
    std::vector<int> part_subgraph_index_; //which subgraph each part belongs to (-1 means none)
    std::vector<double> scores_;

    std::vector<tf::Quaternion> rotations_;
    //std::vector<int> rotation_index;

};
