#ifndef SELECTOR_H
#define SELECTOR_H
/**
 * @copyright
 *
 * Copyright 2012 Kevin Schluff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, 
 * as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace cv;

class Selector
{
public:

    Selector(const char* window)
      :m_selection_valid(false),
       m_selecting(false),
       m_selection(),
       m_origin()
    {
      cv::setMouseCallback(window, mouse_callback, this);
    }

    Selector()
      :m_selection_valid(false),
       m_selecting(false),
       m_selection(),
       m_origin()
    {
    }

    bool valid() const
    { return m_selection_valid; }

    bool selecting() const
    { return m_selecting; }

    const cv::Rect& selection() const
    { return m_selection; }

    bool selections(int num_objects, std::vector<cv::Rect>& output)
    {
        if (num_objects < m_selections.size())
            return false;
        else
        {
            output = m_selections;
            return true;
        }
    }

    int calculate_ranges(Mat HSV, std::vector<Mat>& means, std::vector<Mat>& stds, Mat& mask)
    {//int buffer; std::cin >> buffer;
        for (int i=ranges_calculated_; i<m_selections.size(); i++)
        {
            Mat mean, std;
            meanStdDev(HSV(m_selections[i]), mean, std, mask(m_selections[i]));std::cout<<mean;std::cout<<std;
            means_.push_back(mean);
            stds_.push_back(std);
        }
        ranges_calculated_ = m_selections.size();
        means = means_;
        stds = stds_;
        return ranges_calculated_;
    }

private:
    static void mouse_callback(int event, int x, int y, int flags, void* data);

public:
    int ranges_calculated_ = 0;
    std::vector<Mat> means_;
    std::vector<Mat> stds_;
    bool m_selection_valid;
    bool m_selecting;
    cv::Rect m_selection;
    std::vector<cv::Rect> m_selections;
    cv::Point m_origin;
};

void Selector::mouse_callback(int event, int x, int y, int flags, void* data)
{
   Selector& self = *((Selector*)data);

   switch( event )
   {
      case CV_EVENT_LBUTTONDOWN:
	 self.m_selection_valid = false;
	 self.m_selecting = true;
	 self.m_selection = Rect(0,0,0,0);
	 self.m_origin.x = x;
	 self.m_origin.y = y;
	 break;

      case CV_EVENT_LBUTTONUP:
	 self.m_selection_valid = true;
	 self.m_selecting = false;
     self.m_selections.push_back(self.m_selection);
      default:
	 if( self.m_selecting )
     {
	    self.m_selection.x = MIN(x, self.m_origin.x);
	    self.m_selection.y = MIN(y, self.m_origin.y);
	    self.m_selection.width = std::abs(x - self.m_origin.x);
	    self.m_selection.height = std::abs(y - self.m_origin.y);
	 }
   }
}

#endif
