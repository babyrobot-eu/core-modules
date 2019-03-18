#include <assembly_subtask.hpp>

class AssemblyTask
{

public:
    AssemblyTask(int num_parts,
                 std::vector<std::vector<int>> connnection_pairs,
                 int num_subtasks,
                 std::vector<AssemblySubtask> subtasks,
                 int max_particles,
                 int all_particles,
                 std::vector<std::string> descriptions,
                 bool cubic);
    
    //returns current subtask
    int evaluate_task(std::vector<geometry_msgs::Pose> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list);

    std::string subtask_description(int subtask);

    int num_subtasks_;

private:
    const int num_parts_;
    int current_subtask_ = 0;
    std::vector<AssemblySubtask> subtasks_;
    int max_particles_;
    int all_particles_;
    std::vector<std::string> descriptions_;
    std::vector<std::vector<int>> connection_pairs_;

    int subgraph_max_index_ = -1;
    std::vector<int> part_subgraph_index_; //which subgraph each part belongs to (-1 means none)
    std::vector<tf::Quaternion> rotations_;
    bool cubic_;
};
