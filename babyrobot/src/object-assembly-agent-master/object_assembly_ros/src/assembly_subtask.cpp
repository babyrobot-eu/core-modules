#include <assembly_subtask.hpp>
//TODO rotate desired relative pose, not received poses, to save time
//TODO fix second_rotation (currently only works if desired rotation is zero)

void print_transform(tf::Transform t)
{
    std::cout << t.getBasis().getRow(0).x() << " " << t.getBasis().getRow(0).y() << " " << t.getBasis().getRow(0).z() << " " << t.getOrigin().x() << "\n";
    std::cout << t.getBasis().getRow(1).x() << " " << t.getBasis().getRow(1).y() << " " << t.getBasis().getRow(1).z() << " " << t.getOrigin().y() << "\n";
    std::cout << t.getBasis().getRow(2).x() << " " << t.getBasis().getRow(2).y() << " " << t.getBasis().getRow(2).z() << " " << t.getOrigin().z() << "\n";
    std::cout << "0.0 0.0 0.0 1.0\n";
}

void print_quaternion(tf::Quaternion q)
{
    std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

void print_vector3(tf::Vector3 v)
{
    std::cout << v.x() << " " << v.y() << " " << v.z() << "\n";
}

void print_transform(std::string text, tf::Transform t)
{
    std::cout << text;
    print_transform(t);
}

void print_quaternion(std::string text, tf::Quaternion q)
{
    std::cout << text << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

void print_vector3(std::string text, tf::Vector3 v)
{
    std::cout << text << v.x() << " " << v.y() << " " << v.z() << "\n";
}

AssemblySubtask::AssemblySubtask(
    int id,
    int num_checks,
    int first_object,
    int second_object,
    std::string frame,
    tf::Vector3 relative_position,
    tf::Quaternion relative_orientation,
    std::vector<double> margin,
    std::string connection_type,
    std::vector<std::string> object_symmetries,
    int max_particles,
    tf::Vector3 up_vector,
    int N,
    double ymax)
    : subtask_id_(id),
      num_checks_(num_checks),
      first_object_(first_object),
      second_object_(second_object),
      frame_(frame),
      relative_position_(relative_position),
      relative_orientation_(relative_orientation),
      margin_(margin),
      connection_type_(connection_type),
      max_particles_(max_particles),
      up_vector_(up_vector.normalized()),
      N_(N),
      ymax_(ymax)
{
    //relativeXZ_ = tf::Vector3(relative_position_.getX(), relative_position_.getY(), 0);
    relativeY_ = tf::Vector3(0, relative_position_.getY(), 0);
    relativeXZ_sorted_ = relative_position_.absolute();
    sort_vector3(relativeXZ_sorted_);

    first_object_symmetry_ = object_symmetries[first_object_];
    second_object_symmetry_ = object_symmetries[second_object_];

    //align y axis with down vector
    tf::Vector3 down_vector = up_vector_;
    down_vector *= -1;
    rot_angle_ = down_vector.angle(tf::Vector3(0,1,0));
    if (std::abs(rot_angle_) < 0.001)
        rot_axis_ = tf::Vector3(1.0, 0.0, 0.0);
    else if (std::abs(rot_angle_ - M_PI) < 0.001)
    {
        double x = down_vector.getX();
        double y = down_vector.getY();
        double z = down_vector.getZ();
        if (z != 0 && -x != y)
        {
            rot_axis_ = tf::Vector3(z, z, -x-y);
        }
        else
        {
            rot_axis_ = tf::Vector3(-y-z, x, x);
        }
    }
    else
        rot_axis_ = down_vector.cross(tf::Vector3(0,1,0));
    rot_axis_.normalize();

    tf::Quaternion down_vector_to_y(rot_axis_, -rot_angle_);
    relative_pos_table_frame_ = relative_position_.rotate(rot_axis_, -rot_angle_);
    connection_pose.position.x = relative_pos_table_frame_.x();
    connection_pose.position.y = relative_pos_table_frame_.y();
    connection_pose.position.z = relative_pos_table_frame_.z();
    relative_rot_table_frame_ = relative_orientation_; //down_vector_to_y * relative_orientation_;
    connection_pose.orientation.x = relative_rot_table_frame_.x();
    connection_pose.orientation.y = relative_rot_table_frame_.y();
    connection_pose.orientation.z = relative_rot_table_frame_.z();
    connection_pose.orientation.w = relative_rot_table_frame_.w();
    
    global_to_table_ = tf::Transform::getIdentity();
    global_to_table_.setRotation(tf::Quaternion(rot_axis_, rot_angle_));

    for (int i = 0; i < 24; i++)
    {
        tf::Quaternion temp(cubic_rotations[4*i],cubic_rotations[4*i+1],cubic_rotations[4*i+2],cubic_rotations[4*i+3]);
        cubic_rotation_quaternions_.push_back(temp);
        cubic_relative_positions_.push_back(relative_position_.rotate(temp.getAxis(), temp.getAngle()));
        cubic_relative_positions_normalized_.push_back(cubic_relative_positions_[i].normalized());
    }
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<tf::Transform> &current_object_poses,
        double &score)
{
    tf::Transform first_object_pose, second_object_pose;
    first_object_pose = current_object_poses[first_object_];
    second_object_pose = current_object_poses[second_object_];

    if (evaluate_any(first_object_pose, second_object_pose, score))
    {
        checks_++;
        if (checks_ >= num_checks_)
            subtask_complete_ = true;
    }
    else
    {
        checks_ = 0;
    }

    return subtask_complete_;
}

bool AssemblySubtask::evaluate_subtask(
        std::vector<geometry_msgs::Pose> &current_object_poses,
        double &score)
{
    tf::Transform first_object_pose(tf::Quaternion(
                                        current_object_poses[first_object_].orientation.x,
                                        current_object_poses[first_object_].orientation.y,
                                        current_object_poses[first_object_].orientation.z,
                                        current_object_poses[first_object_].orientation.w),
                                    tf::Vector3(
                                        current_object_poses[first_object_].position.x,
                                        current_object_poses[first_object_].position.y,
                                        current_object_poses[first_object_].position.z));

    tf::Transform second_object_pose(tf::Quaternion(
                                        current_object_poses[second_object_].orientation.x,
                                        current_object_poses[second_object_].orientation.y,
                                        current_object_poses[second_object_].orientation.z,
                                        current_object_poses[second_object_].orientation.w),
                                     tf::Vector3(
                                        current_object_poses[second_object_].position.x,
                                        current_object_poses[second_object_].position.y,
                                        current_object_poses[second_object_].position.z));

    if (evaluate_any(first_object_pose, second_object_pose, score))
    {
        checks_++;
        if (checks_ >= num_checks_)
            subtask_complete_ = true;
    }
    else
    {
        checks_ = 0;
    }

    return subtask_complete_;
}

bool AssemblySubtask::evaluate_any(tf::Transform &first_object_pose,
                                   tf::Transform &second_object_pose,
                                   double &score)
{
    if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "cubic")
    {
        return evaluate_cubic_cubic(first_object_pose,
                                    second_object_pose,
                                    score);
    }
    else if (first_object_symmetry_ == "none" &&
            second_object_symmetry_ == "cubic")
    {
        return evaluate_none_cubic(first_object_pose,
                                   second_object_pose,
                                   score);
    }
    else if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "none")
    {
        return evaluate_cubic_none(first_object_pose,
                                   second_object_pose,
                                   score);
    }
    else
    {
        return evaluate_no_symmetry(first_object_pose,
                                    second_object_pose,
                                    score);
    }
}

/* Returns number of permutations performed */
int AssemblySubtask::sort_vector3(tf::Vector3 &v)
{
    int perms = 0;
    double temp, x, y, z;
    x = v.getX();
    y = v.getY();
    z = v.getZ();
    if (x > z)
    {
        temp = x;
        x = z;
        z = temp;
        ++perms;
    }
    if (x > y)
    {
        temp = x;
        x = y;
        y = temp;
        ++perms;
    }
    if (y > z)
    {
        temp = y;
        y = z;
        z = temp;
        ++perms;
    }
    v.setX(x);
    v.setY(y);
    v.setZ(z);
    return perms;
}

bool AssemblySubtask::evaluate_no_symmetry(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check1, position_check2, orientation_check = true;
    double position_score1, position_score2, orientation_score = 1.0;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation1_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);
    tf::Vector3 position_diff1 = t21.getOrigin();
    position_check1 = compare_tf_vectors(relative_position_,
                                         position_diff1,
                                         margin_[0],
                                         margin_[1],
                                         margin_[2],
                                         position_score1);

    tf::Transform rotated_second_object_pose = second_object_pose *
                                               tf::Transform(prerotation2_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t12(rotated_second_object_pose);
    t12 = t12.inverseTimes(first_object_pose);
    tf::Vector3 position_diff2 = t12.getOrigin().rotate(relative_orientation_.getAxis(), relative_orientation_.getAngle());
    position_check2 = compare_tf_vectors(-relative_position_,
                                         position_diff2,
                                         margin_[0],
                                         margin_[1],
                                         margin_[2],
                                         position_score2);

    set_connection_pose();
    score = position_score1; // * orientation_score);
    return position_check1 && position_check2 && orientation_check;
}

bool AssemblySubtask::evaluate_cubic_cubic(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool positionXZ_check, positionY_check, orientation_check = true;
    double positionXZ_score, positionY_score, orientation_score = 1.0;

    tf::Transform t21(global_to_table_*first_object_pose);
    t21 = t21.inverseTimes(global_to_table_*second_object_pose);
    tf::Vector3 position_diff = t21.getOrigin();
    double max_dot_prod = std::numeric_limits<double>::lowest();
    int max_index = 0;
    for (int i = 0; i < 24; i++)
    {
        double dot_prod = position_diff.x() * cubic_relative_positions_normalized_[i].x() + 
                          position_diff.y() * cubic_relative_positions_normalized_[i].y() + 
                          position_diff.z() * cubic_relative_positions_normalized_[i].z();
        if (dot_prod > max_dot_prod)
        {
            max_dot_prod = dot_prod;
            max_index = i;
        }
    }
    positionXZ_check = compare_tf_vectors(cubic_relative_positions_[max_index],
                                          position_diff,
                                          margin_[0],
                                          margin_[1],
                                          margin_[2],
                                          positionXZ_score);

    tf::Vector3 first_pos = first_object_pose.getOrigin();
    first_pos.rotate(rot_axis_, rot_angle_);
    tf::Vector3 second_pos = second_object_pose.getOrigin();
    second_pos.rotate(rot_axis_, rot_angle_);
    positionY_check = compare_tf_vectors(relativeY_,
                                        tf::Vector3(0, second_pos.getY() - first_pos.getY(), 0),
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        positionY_score);


        tf::Vector3 down_vector_in_frame;
        
        first_rotation = cubic_rotation_quaternions_[max_index].inverse();
        down_vector_in_frame = (global_to_table_*first_object_pose).inverse() * (tf::Vector3(0.0, 1.0, 0.0)) - (global_to_table_*first_object_pose).inverse().getOrigin(); 
        down_vector_in_frame = down_vector_in_frame.rotate(first_rotation.getAxis(), first_rotation.getAngle());
        if (std::abs(down_vector_in_frame.angle(tf::Vector3(0.0, 1.0, 0.0)) - M_PI) < 0.001)
        {
            first_rotation = tf::Quaternion(relative_position_, M_PI) * first_rotation;
        }
        else if (!std::abs(down_vector_in_frame.angle(tf::Vector3(0.0, 1.0, 0.0))) < 0.001)
        {
            first_rotation = tf::Quaternion(down_vector_in_frame.cross(tf::Vector3(0.0, 1.0, 0.0)), down_vector_in_frame.angle(tf::Vector3(0.0, 1.0, 0.0))) * first_rotation;
        }        
        first_rotation = round_quaternion_90(first_rotation);
        second_rotation = round_quaternion_90(t21.getRotation());
        set_connection_pose();

    score = positionXZ_score;
    return positionXZ_check;
//TODO fix positionY_check
}


bool AssemblySubtask::evaluate_cubic_none(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check, positionY_check = 1, orientation_check;
    double position_score, positionY_score = 1.0, orientation_score;

    tf::Transform rotated_second_object_pose = second_object_pose *
                                               tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform t21(rotated_second_object_pose);
    t21 = t21.inverseTimes(first_object_pose);

    tf::Vector3 position_diff = t21.getOrigin();
    position_check = compare_tf_vectors(-relative_position_,
                                          position_diff,
                                          margin_[0],
                                          margin_[1],
                                          margin_[2],
                                          position_score);

    if (position_score >= 1.0/max_particles_)
    {
        first_rotation = round_quaternion_90(second_object_pose.inverseTimes(first_object_pose).getRotation());
        set_connection_pose();
    }

    orientation_score = 1.0;
    orientation_check = true;
    score = position_score; // * positionY_score); // * orientation_score;
    return position_check;// && positionY_check && orientation_check;
}

bool AssemblySubtask::evaluate_none_cubic(
                                  tf::Transform &first_object_pose,
                                  tf::Transform &second_object_pose,
                                  double &score)
{
    bool position_check, positionZ_check = true, orientation_check;
    double position_score, positionZ_score = 1.0, orientation_score;

    tf::Transform rotated_first_object_pose = first_object_pose * 
                                              tf::Transform(prerotation_, tf::Vector3(0.0, 0.0, 0.0));

    tf::Transform t21(rotated_first_object_pose);
    t21 = t21.inverseTimes(second_object_pose);

    tf::Vector3 position_diff = t21.getOrigin();

    position_check = compare_tf_vectors(relative_position_,
                                        position_diff,
                                        margin_[0],
                                        margin_[1],
                                        margin_[2],
                                        position_score);
    if (position_score >= 1.0/max_particles_)
    {
        second_rotation = round_quaternion_90(first_object_pose.inverseTimes(second_object_pose).getRotation());
        set_connection_pose();
    }

    orientation_score = 1.0;
    orientation_check = true;
    score = position_score; // * positionZ_score); // * orientation_score;
    return position_check;// && positionZ_check && orientation_check;
}

bool AssemblySubtask::compare_tf_vectors(tf::Vector3 v1,
                                         tf::Vector3 v2,
                                         double x_margin,
                                         double y_margin,
                                         double z_margin,
                                         double &score)
{
    const static double sqrt3 = 1.732050808;
    score = std::min(1.0, sqrt3/std::sqrt( ((ymax_-1)/(N_*N_-1)) * (pow((v1.getX() - v2.getX())/(x_margin), 2) + pow((v1.getY() - v2.getY())/(y_margin), 2) + pow((v1.getZ() - v2.getZ())/(z_margin), 2) ) + 3*(N_*N_-ymax_)/(N_*N_-1) ) );
    return (std::abs(v1.getX() - v2.getX()) < x_margin &&
            std::abs(v1.getY() - v2.getY()) < y_margin &&
            std::abs(v1.getZ() - v2.getZ()) < z_margin);
}

bool AssemblySubtask::compare_tf_quaternions(tf::Quaternion q1,
                                             tf::Quaternion q2,
                                             double margin,
                                             double &score)
{
    score = 1; //TODO
    return double(std::abs(q1.dot(q2)) > 1 - margin);
}

void AssemblySubtask::set_first_object_symmetry(std::string new_type)
{
    first_object_symmetry_ = new_type;
}

void AssemblySubtask::set_second_object_symmetry(std::string new_type)
{
    second_object_symmetry_ = new_type;
}

std::string AssemblySubtask::get_first_object_symmetry()
{
    return first_object_symmetry_;
}

std::string AssemblySubtask::get_second_object_symmetry()
{
    return second_object_symmetry_;
}

void AssemblySubtask::set_prerotation(tf::Quaternion rotation)
{
    //prerotation_ = tf::Quaternion::getIdentity();
    prerotation_ = rotation;
}

void AssemblySubtask::set_prerotations(tf::Quaternion rotation1, tf::Quaternion rotation2)
{
    prerotation1_ = rotation1;
    prerotation2_ = rotation2;
}

tf::Quaternion AssemblySubtask::round_quaternion_90(tf::Quaternion rotation)
{
    double x, y, z, w;

    double max_dot_prod = -4; //std::numeric_limits<double>::infinity();
    int max_index = 0;
    for (int i = 0; i < 24; i++)
    {
        double dot_prod = std::abs(cubic_rotations[4*i]*rotation.x() + cubic_rotations[4*i+1]*rotation.y() + 
                          cubic_rotations[4*i+2]*rotation.z() + cubic_rotations[4*i+3]*rotation.w());
        if (dot_prod > max_dot_prod)
        {
            max_dot_prod = dot_prod;
            max_index = i;
        }
    }
    x = cubic_rotations[4*max_index];
    y = cubic_rotations[4*max_index+1];
    z = cubic_rotations[4*max_index+2];
    w = cubic_rotations[4*max_index+3];

    tf::Quaternion rounded_rotation(x, y, z, w);
    rounded_rotation.normalize();
    return rounded_rotation;
}

void AssemblySubtask::set_connection_pose()
{
    tf::Vector3 rotated_relative_pos;
    tf::Quaternion rotated_relative_rot;

    if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "cubic")
    {
        tf::Quaternion first_inverted = first_rotation.inverse();
        rotated_relative_pos = relative_position_.rotate(first_inverted.getAxis(), first_inverted.getAngle());
        rotated_relative_rot = second_rotation;
    }
    else if (first_object_symmetry_ == "none" &&
            second_object_symmetry_ == "cubic")
    {
        rotated_relative_pos = relative_position_.rotate(prerotation_.getAxis(), prerotation_.getAngle());
        rotated_relative_rot = second_rotation;
    }
    else if (first_object_symmetry_ == "cubic" &&
            second_object_symmetry_ == "none")
    {
        tf::Quaternion first_inverted = first_rotation.inverse();
        rotated_relative_pos = relative_position_.rotate((first_inverted*prerotation_).getAxis(), (first_inverted*prerotation_).getAngle());
        rotated_relative_rot = first_inverted;
    }
    else
    {
        rotated_relative_pos = relative_position_.rotate(prerotation1_.getAxis(), prerotation1_.getAngle());
        rotated_relative_rot = prerotation1_ * prerotation2_.inverse() * relative_orientation_; //TODO
    }

    connection_pose.position.x = rotated_relative_pos.x();
    connection_pose.position.y = rotated_relative_pos.y();
    connection_pose.position.z = rotated_relative_pos.z();

    connection_pose.orientation.x = rotated_relative_rot.x();
    connection_pose.orientation.y = rotated_relative_rot.y();
    connection_pose.orientation.z = rotated_relative_rot.z();
    connection_pose.orientation.w = rotated_relative_rot.w();

}
