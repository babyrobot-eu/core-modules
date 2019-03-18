#include <assembly_task_guess_mode.hpp>

AssemblyTask::AssemblyTask(
    int id,
    int num_parts,
    std::vector<std::vector<int>> connection_pairs,
    int num_connections,
    std::vector<AssemblySubtask> subtasks,
    int max_particles,
    int all_particles,
    int connection_list_offset,
    std::string connection_vector_topic
    )
    : node_handle_("~"),
      task_id_(id),
      num_parts_(num_parts),
      connection_pairs_(connection_pairs),
      num_connections_(num_connections),
      subtasks_(subtasks),
      max_particles_(max_particles),
      all_particles_(all_particles),
      connection_list_offset_(connection_list_offset)
{
    tf::Quaternion unit_quaternion(0.0, 0.0, 0.0, 1.0);
    for (int i = 0; i < num_parts_; i++)
    {
        rotations_.push_back(unit_quaternion);
        part_subgraph_index_.push_back(-1);
    }
    for (int i = 0; i < num_connections_; i++)
    {
        connection_status_vector_.push_back(false);
        zero_score_queue_.push_back(i);
        zero_score_spotlight_times_.push_back(0);
    }
    scores_ = std::vector<double>(num_connections,0);
    connection_vector_publisher_ = node_handle_.advertise<object_assembly_msgs::ConnectionVector>(connection_vector_topic, 0);
}

double AssemblyTask::evaluate_task(std::vector<tf::Transform> current_object_poses, std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{
    for (int i = 0; i < subtasks_.size(); i++)
    {
        if (!connection_status_vector_[i])
        {
            if (part_subgraph_index_[connection_pairs_[i][0]] == -1 &&
                part_subgraph_index_[connection_pairs_[i][1]] == -1)
            {
                //neither part is connected to a third part
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    rotations_[connection_pairs_[i][0]] = subtasks_[i].first_rotation;
                    rotations_[connection_pairs_[i][1]] = subtasks_[i].first_rotation * subtasks_[i].second_rotation;
                    rotations_[connection_pairs_[i][1]].normalize();
                    part_subgraph_index_[connection_pairs_[i][0]] = ++subgraph_max_index_;
                    part_subgraph_index_[connection_pairs_[i][1]] = subgraph_max_index_;
                }
            }
            else if (part_subgraph_index_[connection_pairs_[i][1]] == -1)
            {
                //first part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[i][0]];
                subtasks_[i].set_first_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[i][0]].inverse();
                subtasks_[i].set_prerotation(prerotation);
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    rotations_[connection_pairs_[i][1]] = rotations_[connection_pairs_[i][0]] * subtasks_[i].second_rotation;
                    part_subgraph_index_[connection_pairs_[i][1]] = subgraph_ind;
                }
            }
            else if(part_subgraph_index_[connection_pairs_[i][0]] == -1)
            {
                //second part is already connected to a third part
                int subgraph_ind = part_subgraph_index_[connection_pairs_[i][1]];
                subtasks_[i].set_second_object_symmetry("none");
                tf::Quaternion prerotation = rotations_[connection_pairs_[i][1]].inverse();
                subtasks_[i].set_prerotation(prerotation);
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    rotations_[connection_pairs_[i][0]] = rotations_[connection_pairs_[i][1]] * subtasks_[i].first_rotation;
                    part_subgraph_index_[connection_pairs_[i][0]] = subgraph_ind;
                }
            }
            else
            {
                //both parts are connected to other components
                //Some clutter will be created, but the possible subgraphs are limited in number, so the effect is negligible
                subtasks_[i].set_first_object_symmetry("none");
                subtasks_[i].set_second_object_symmetry("none");
                tf::Quaternion prerotation1 = rotations_[connection_pairs_[i][0]].inverse();
                tf::Quaternion prerotation2 = rotations_[connection_pairs_[i][1]].inverse();
                subtasks_[i].set_prerotations(prerotation1, prerotation2);
                connection_status_vector_[i] = subtasks_[i].evaluate_subtask(current_object_poses, scores_[i]);
                if (connection_status_vector_[i])
                {
                    int subgraph_ind0 = part_subgraph_index_[connection_pairs_[i][0]];
                    int subgraph_ind1 = part_subgraph_index_[connection_pairs_[i][1]];
                    for (int j = 0; j < num_parts_; j++)
                    {
                        if (part_subgraph_index_[j] == subgraph_ind1)
                            part_subgraph_index_[j] = subgraph_ind0;
                    }
                }
            }
        }
    }
    set_connections(connection_list);

    int count_ones = 0;
    std::cout << "Status Vector: [";
    for (int i = 0; i < num_connections_; i++)
    {
        std::cout << connection_status_vector_[i] << ", ";
        if (connection_status_vector_[i]) count_ones++;
    }
    std::cout << "\b\b  \b\b]\n";
    publish_connection_vector();

    return double(count_ones)/double(num_connections_);
}

void AssemblyTask::set_connections(std::vector<object_assembly_msgs::ConnectionInfo> &connection_list)
{

    static int conn_method = 0;
    if(conn_method == 0)
    {
        for (int i = 0; i < num_connections_; i++)
        {
            if (connection_status_vector_[i])
            {
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = all_particles_;
                connection_list[connection_list_offset_ + i].first_particle = 0;
            }
            else if (scores_[i] >= 1.0/max_particles_)
            {
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = int(scores_[i] * max_particles_);// < max_particles_ ? int(scores_[i] * max_particles_) + 1 : max_particles_;
            }
            else
            {
                connection_list[connection_list_offset_ + i].num_particles = 0;
            }
        }
    }

    if (conn_method == 1)
    {
        int sum_none_zero = 0;
        int num_zeros = 0;
        std::vector<bool> none_zero_index;
           std::vector<int> none_zeros;
        for (int i = 0; i < num_connections_; i++)
        {
            if (connection_status_vector_[i])
            {
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = all_particles_;
            }
            else if (scores_[i] >= 1.0/max_particles_)
            {
                none_zero_index.push_back(true);
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = int(scores_[i] * max_particles_);// < max_particles_ ? int(scores_[i] * max_particles_) + 1 : max_particles_;
                sum_none_zero += int(scores_[i] * max_particles_);
                for (int j = 0; j < int(scores_[i] * max_particles_); j++)
                    none_zeros.push_back(i);
            }
            else
            {
                none_zero_index.push_back(false);
                connection_list[connection_list_offset_ + i].num_particles = 0;
                ++num_zeros;
            }
        }

        //Remove excess particles at random
        if (sum_none_zero > max_particles_)
        {
            std::random_shuffle(none_zeros.begin(), none_zeros.end());
        }
        for (int i = 0; i < sum_none_zero - max_particles_ ; i++)
        {
            if (--connection_list[connection_list_offset_ + none_zeros[i]].num_particles == 0)
                ++num_zeros;
        }

        //Asign free particles with random connections
        for (int i = 0; i < std::min(max_particles_ - sum_none_zero, num_zeros); i++)
        {
            int ind;
            do
            {
                ind = std::rand()%num_connections_;
            } while (connection_list[connection_list_offset_ + ind].num_particles != 0);
            connection_list[connection_list_offset_ + ind].num_particles = 1;
            //connection_list[connection_list_offset_ + ind].first_particle = 1;
        }

        //Fix particle ranges
        int particle_offset = task_id_ * all_particles_;
        for (int i = 0; i < num_connections_; i++)
        {
            if (connection_status_vector_[i])
                connection_list[connection_list_offset_ + i].first_particle = task_id_ * all_particles_;
            else
            {
                connection_list[connection_list_offset_ + i].first_particle = particle_offset;
                particle_offset += connection_list[connection_list_offset_ + i].num_particles;
            }
        }
    }

    if (conn_method == 2)
    {
        int sum_none_zero = 0;
        int num_zeros = 0;
        std::vector<bool> none_zero_index;
           std::vector<int> none_zeros;
        std::list<int> new_queue;
        //std::list<int> new_spotlight_times;
        auto new_queue_split_point = new_queue.begin(); //TODO change name
        //auto new_spotlight_times_top = new_spotlight_times.begin();
        bool queue_back_end = false;
        for (int i : zero_score_queue_)
        {
            if (connection_status_vector_[i])
            {
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = all_particles_;
            }
            else if (scores_[i] >= 1.0/max_particles_)
            {
                none_zero_index.push_back(true);
                connection_list[connection_list_offset_ + i].relative_pose = subtasks_[i].connection_pose;
                connection_list[connection_list_offset_ + i].num_particles = int(scores_[i] * max_particles_);// < max_particles_ ? int(scores_[i] * max_particles_) + 1 : max_particles_;
                sum_none_zero += int(scores_[i] * max_particles_);
                for (int j = 0; j < int(scores_[i] * max_particles_); j++)
                    none_zeros.push_back(i);
                new_queue.push_back(i);
                zero_score_spotlight_times_[i] = 0;
                if (!queue_back_end)
                {
                    --new_queue_split_point;
                    queue_back_end = true;
                }
            }
            else
            {
                none_zero_index.push_back(false);
                connection_list[connection_list_offset_ + i].num_particles = 0;
                ++num_zeros;
                if (zero_score_spotlight_times_[i] >= 5) //TODO pass as param
                {
                    new_queue.push_back(i);
                    zero_score_spotlight_times_[i] = 0;
                    if (!queue_back_end)
                    {
                        --new_queue_split_point;
                        queue_back_end = true;
                    }
                }
                else
                {
                    
                    new_queue.insert(new_queue_split_point, i);
                }
            }

        }

        zero_score_queue_ = new_queue;

        //Remove excess particles at random
        if (sum_none_zero > max_particles_)
        {
            std::random_shuffle(none_zeros.begin(), none_zeros.end());
        }
        for (int i = 0; i < sum_none_zero - max_particles_ ; i++)
        {
            if (--connection_list[connection_list_offset_ + none_zeros[i]].num_particles == 0)
                ++num_zeros;
        }

        //Asign free particles with random connections
        for (int i = 0; i < std::min(max_particles_ - sum_none_zero, num_zeros); i++)
        {

            int ind = new_queue.front();
            connection_list[connection_list_offset_ + ind].num_particles = 1;
            ++zero_score_spotlight_times_[ind];
            new_queue.pop_front();
        }

        //Fix particle ranges
        int particle_offset = task_id_ * all_particles_;
        for (int i = 0; i < num_connections_; i++)
        {
            if (connection_status_vector_[i])
                connection_list[connection_list_offset_ + i].first_particle = task_id_ * all_particles_;
            else
            {
                connection_list[connection_list_offset_ + i].first_particle = particle_offset;
                particle_offset += connection_list[connection_list_offset_ + i].num_particles;
            }
        }
    }
}

void AssemblyTask::publish_connection_vector()
{
    object_assembly_msgs::ConnectionVector connection_vector_msg;
    for (int i = 0; i < connection_status_vector_.size(); i++)
        connection_vector_msg.v.push_back((char)connection_status_vector_[i]);
    connection_vector_publisher_.publish(connection_vector_msg);
}
