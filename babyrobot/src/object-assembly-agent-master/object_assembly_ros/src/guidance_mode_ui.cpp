#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <base_ui.hpp>

class GuessModeUI: public BaseUI
{
public:
    GuessModeUI(std::vector<std::string> filenames);
    void set_current_part(int part);
    void logic();
    void draw();
    void onDisplay();
    void onIdle();
private:
    int current_part_ = 0;
    float total_time_ = 0.0;
};

GuessModeUI *ui_global;

void BaseUI::onDisplay_static()
{
    ui_global->onDisplay();
}

void BaseUI::onReshape_static(int width, int height) 
{
    ui_global->onReshape(width, height);
}

void BaseUI::onIdle_static()
{
    ui_global->onIdle();
}

GuessModeUI::GuessModeUI(std::vector<std::string> filenames) : BaseUI(filenames)
{
    char temp_name[] = "Guess Mode";
    std::strcpy(mode_name_, temp_name);
}

void GuessModeUI::set_current_part(int part)
{
    current_part_ = part;
}

void GuessModeUI::logic()
{
    /* FPS count */
    fps_frames++;
    int delta_t = glutGet(GLUT_ELAPSED_TIME) - fps_start;
    if (delta_t > 1000) {
        std::cout << 1000.0 * fps_frames / delta_t << std::endl;
        fps_frames = 0;
        fps_start = glutGet(GLUT_ELAPSED_TIME);
    }

    // Model
    // Set in onDisplay() - cf. main_object.object2world

    // View
    glm::mat4 world2camera = transforms[MODE_CAMERA];

    // Projection
    glm::mat4 camera2screen = glm::perspective(45.0f, 1.0f*screen_width/screen_height, 0.1f, 100.0f);

    glUseProgram(program);
    glUniformMatrix4fv(uniform_v, 1, GL_FALSE, glm::value_ptr(world2camera));
    glUniformMatrix4fv(uniform_p, 1, GL_FALSE, glm::value_ptr(camera2screen));

    glm::mat4 v_inv = glm::inverse(world2camera);
    glUniformMatrix4fv(uniform_v_inv, 1, GL_FALSE, glm::value_ptr(v_inv));

    glutPostRedisplay();
}

void GuessModeUI::draw() {
    glClearColor(0.45, 0.45, 0.45, 1.0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glUseProgram(program);

    for (int i = 0; i < meshes_.size(); i++)
        meshes_[i].draw();
    ground.draw();
    //light_bbox.draw_bbox();
}

void GuessModeUI::onDisplay()
{
    ui_global->logic();
    ui_global->draw();
    glutSwapBuffers();
}

void GuessModeUI::onIdle()
{
    total_time_ = glutGet(GLUT_ELAPSED_TIME);
    float new_red = 0.25*std::sin(6.28*total_time_ / 1000.0) + 0.25;  // 2 second period
std::cout << "red: " << new_red << "\n";
    
    meshes_[current_part_].set_uniform_colour(glm::vec3(new_red,meshes_[current_part_].colours[0][1],meshes_[current_part_].colours[0][2]));
    glUseProgram(program);
    //glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
    glutPostRedisplay();
}

int main(int argc, char* argv[])
{
    std::vector<std::string> filenames;
    filenames.push_back("/home/jack/catkin_ws/src/object_assembly_ros/resource/cube.obj");
    filenames.push_back("/home/jack/catkin_ws/src/object_assembly_ros/resource/cube.obj");
    filenames.push_back("/home/jack/catkin_ws/src/object_assembly_ros/resource/cube.obj");
    filenames.push_back("/home/jack/catkin_ws/src/object_assembly_ros/resource/cube.obj");std::cout << "1\n";
    GuessModeUI ui(filenames);
    ui_global = &ui;
    tf::Transform t1;
    t1.setIdentity();
    t1.setOrigin(tf::Vector3(0.1,0.1,0.6));
    std::vector<tf::Transform> new_poses;
    new_poses.push_back(t1);
    t1.setOrigin(tf::Vector3(-0.1,0.1,0.6));
    new_poses.push_back(t1);
    t1.setOrigin(tf::Vector3(0.1,-0.1,0.6));
    new_poses.push_back(t1);
    t1.setOrigin(tf::Vector3(-0.1,-0.1,0.6));
    new_poses.push_back(t1);
    ui.update_poses(new_poses);
    for (int i = 0; i < ui.meshes_.size(); i++)
    {
        ui.meshes_[i].set_uniform_colour(glm::vec3(0.0,0.0,0.3));
    }
    ui.gui_main(argc, argv);
}

