#ifndef LEARNING_MODE_UI_HPP
#define LEARNING_MODE_UI_HPP

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <base_ui.hpp>

class LearningModeUI: public BaseUI
{
public:
    LearningModeUI(std::vector<std::string> filenames);
    void update_probabilities();
    void logic();
    void draw();
    void onDisplay();
    void onIdle();
};

LearningModeUI *ui_global;

#endif //LEARNING_MODE_UI_HPP
