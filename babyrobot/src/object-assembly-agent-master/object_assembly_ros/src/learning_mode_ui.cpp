#include <learning_mode_ui.hpp>


int BaseUI::gui_main_static(int argc, char* argv[])
{
    ui_global->gui_main(argc, argv);
}

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

LearningModeUI::LearningModeUI(std::vector<std::string> filenames) : BaseUI(filenames)
{
    char temp_name[] = "Learning Mode";
    std::strcpy(mode_name_, temp_name);
    ui_global = this;
}

void LearningModeUI::update_probabilities()
{

}

void LearningModeUI::logic()
{
    /* FPS count */
    fps_frames++;
    int delta_t = glutGet(GLUT_ELAPSED_TIME) - fps_start;
    if (delta_t > 1000) {
        //std::cout << 1000.0 * fps_frames / delta_t << std::endl;
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

void LearningModeUI::draw() {
    
    glClearColor(0.45, 0.45, 0.45, 1.0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glUseProgram(program);

    for (int i = 0; i < meshes_.size(); i++)
        meshes_[i].draw();
    //ground.draw();
    //light_bbox.draw_bbox();
}

void LearningModeUI::onDisplay()
{
    std::lock_guard<std::mutex> lock_mutex(mutex_);
    ui_global->logic();
    ui_global->draw();
    glutSwapBuffers();
}

void LearningModeUI::onIdle()
{

}

