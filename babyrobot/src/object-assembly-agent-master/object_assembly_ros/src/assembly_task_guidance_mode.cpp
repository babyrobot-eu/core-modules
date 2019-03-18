#include <assembly_task_guidance_mode.hpp>

AssemblyTask::AssemblyTask(
    int num_parts,
    std::vector<std::vector<int>> connection_pairs,
    int num_subtasks,
    std::vector<AssemblySubtask> subtasks,
    int max_particles,
    int all_particles,
    std::vector<std::string> descriptions,
    bool cubic
    )
    : num_parts_(),
      connection_pairs_(connection_pairs),
      num_subtasks_(num_subtasks),
      subtasks_(subtasks),
      max_particles_(max_particles),
      all_particles_(all_particles),
      descriptions_(descriptions),
      cubic_(cubic)
{
    tf::Quaternion unit_quaternion(0.0, 0.0, 0.0, 1.0);
    for (int i = 0; i < num_parts_; i++)
    {
        rotations_.push_back(unit_quaternion);
        part_subgraph_index_.push_back(-1);
    }
}

int AssemblyTask::evaluate_task(std::vector<geometry_msgs::Pose> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{
    if (current_subtask_==num_subtasks_)
    {
        //Assembly finished! Return number of subtasks
        return current_subtask_;
    }
    double score;

    while (subtasks_[current_subtask_].evaluate_subtask(current_object_poses, score))
    {
        if (cubic_)
        {
            //Update rotations and subgraph indices
            if (part_subgraph_index_[connection_pairs_[current_subtask_][0]] == -1 &&
                part_subgraph_index_[connection_pairs_[current_subtask_][1]] == -1)
            {
                rotations_[connection_pairs_[current_subtask_][0]] = subtasks_[current_subtask_].first_rotation;
                rotations_[connection_pairs_[current_subtask_][1]] = subtasks_[current_subtask_].first_rotation * subtasks_[current_subtask_].second_rotation;
                rotations_[connection_pairs_[current_subtask_][1]].normalize();
                part_subgraph_index_[connection_pairs_[current_subtask_][0]] = ++subgraph_max_index_;
                part_subgraph_index_[connection_pairs_[current_subtask_][1]] = subgraph_max_index_;
            }
            else if (part_subgraph_index_[connection_pairs_[current_subtask_][1]] == -1)
            {
                rotations_[connection_pairs_[current_subtask_][1]] = rotations_[connection_pairs_[current_subtask_][0]] * subtasks_[current_subtask_].second_rotation;
                part_subgraph_index_[connection_pairs_[current_subtask_][1]] = part_subgraph_index_[connection_pairs_[current_subtask_][0]];;
            }
            else if(part_subgraph_index_[connection_pairs_[current_subtask_][0]] == -1)
            {
                rotations_[connection_pairs_[current_subtask_][0]] = rotations_[connection_pairs_[current_subtask_][1]] * subtasks_[current_subtask_].first_rotation;
                part_subgraph_index_[connection_pairs_[current_subtask_][0]] = part_subgraph_index_[connection_pairs_[current_subtask_][1]];
            }
            else
            {
                int subgraph_ind0 = part_subgraph_index_[connection_pairs_[current_subtask_][0]];
                int subgraph_ind1 = part_subgraph_index_[connection_pairs_[current_subtask_][1]];
                for (int j = 0; j < num_parts_; j++)
                {
                    if (part_subgraph_index_[j] == subgraph_ind1)
                        part_subgraph_index_[j] = subgraph_ind0;
                }
            }
        }

        //Set connection info
        connection_list[current_subtask_].relative_pose = subtasks_[current_subtask_].connection_pose;
        connection_list[current_subtask_].first_particle = 0;
        connection_list[current_subtask_].num_particles = all_particles_;
        current_subtask_++;
        if (current_subtask_==num_subtasks_)
        {
            //Assembly finished! Return number of subtasks
            return current_subtask_;
        }

        if (cubic_)
        {
            //Set prerotations and object symmetries
            if (part_subgraph_index_[connection_pairs_[current_subtask_][0]] == -1 &&
                part_subgraph_index_[connection_pairs_[current_subtask_][1]] == -1)
            {
                //neither part is connected to a third part
            }
            else if (part_subgraph_index_[connection_pairs_[current_subtask_][1]] == -1)
            {
                //first part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[current_subtask_][0]];
                subtasks_[current_subtask_].set_first_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[current_subtask_][0]].inverse();
                subtasks_[current_subtask_].set_prerotation(prerotation);
            }
            else if(part_subgraph_index_[connection_pairs_[current_subtask_][0]] == -1)
            {
                //second part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[current_subtask_][1]];
                subtasks_[current_subtask_].set_second_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[current_subtask_][1]].inverse();
                subtasks_[current_subtask_].set_prerotation(prerotation);
            }
            else
            {
                //both parts are connected to other components
                //Some clutter will be created, but the possible subgraphs are limited in number, so the effect is negligible
                subtasks_[current_subtask_].set_first_object_symmetry("none");
                subtasks_[current_subtask_].set_second_object_symmetry("none");
                tf::Quaternion prerotation1 = rotations_[connection_pairs_[current_subtask_][0]].inverse();
                tf::Quaternion prerotation2 = rotations_[connection_pairs_[current_subtask_][1]].inverse();
                subtasks_[current_subtask_].set_prerotations(prerotation1, prerotation2);
            }
        }
    }
    connection_list[current_subtask_].relative_pose = subtasks_[current_subtask_].connection_pose;
    connection_list[current_subtask_].num_particles = int(score * max_particles_);
    return current_subtask_;
}

std::string AssemblyTask::subtask_description(int subtask)
{
    return descriptions_[subtask];
}
