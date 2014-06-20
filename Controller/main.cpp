#include "../Model/FeatureFinder.h"
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "FeatureFinder");

    // ros::start();

    cv::namedWindow("CVWindowName");

    FeatureFinder * finder = new FeatureFinder("CVWindowName");
    finder->enableVideoMode();

    cv::waitKey(2);
    
    while(ros::ok())
        ros::spinOnce();

    cv::destroyWindow(finder->WINDOW_NAME);

    return 0;
}