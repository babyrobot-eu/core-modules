#ifndef GUESS_MODE_HPP
#define GUESS_MODE_HPP

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
    void update_probabilities();
    void logic();
    void draw();
    void onDisplay();
    void onIdle();
};

GuessModeUI *ui_global;

#endif //GUESS_MODE_HPP
