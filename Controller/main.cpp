#include "../Model/FeatureFinder.h"
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "FeatureFinder");

    ros::start();

    FeatureFinder * finder = new FeatureFinder();
    finder->enableVideoMode();

    while(ros::ok())
    {
       ros::spinOnce();;
    }
    //cv::destroyWindow(finder->WINDOW_NAME);

    return 0;
}