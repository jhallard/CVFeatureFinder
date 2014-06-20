#include "../Model/FeatureFinder.h"
#include <iostream>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "FeatureFinder");

    FeatureFinder * finder = new FeatureFinder("CVWindowName");
    finder->enableVideoMode();

    if(argc == 2)
    {
        if(argv[1][0] == 'l')
            finder->getLeft()->toggleVideoMode(true);
        else if(argv[1][0] == 'r')
            finder->getRight()->toggleVideoMode(true);
        else
        {
            finder->getLeft()->toggleVideoMode(true);
            finder->getRight()->toggleVideoMode(true);
        }
    }

    
    while(ros::ok())
        ros::spinOnce();

    cv::destroyWindow(finder->WINDOW_NAME);

    return 0;
}